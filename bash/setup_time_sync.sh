#!/usr/bin/env bash
# Time-sync stack for the mapping rig (run with sudo on EACH Jetson).
#
# Two-Jetson topology (one PTP domain on a shared GbE switch):
#   master  -> the CAMERA/GNSS Jetson. chrony serves a (free-running) clock to
#              the LAN; ptp4l is PTP grandmaster so the LiDAR Jetson AND both
#              LiDARs discipline to it. This is also where a future GNSS/PPS
#              upgrade plugs in (see GNSS UPGRADE at the bottom).
#   slave   -> the LiDAR Jetson. ptp4l slaveOnly + phc2sys discipline this
#              Jetson's system clock to the grandmaster. The LiDARs lock to the
#              same grandmaster independently (Hesai web UI / Livox auto).
#
# Single-Jetson use: run with role 'master' only (original behaviour).
#
# Usage:  sudo bash setup_time_sync.sh <master|slave> [IFACE] [MASTER_IP]
#         sudo bash setup_time_sync.sh master eth0
#         sudo bash setup_time_sync.sh slave  eth0 192.168.1.50
#
# Verify a recording afterwards:
#   python3 evaluation/check_time_sync.py --bag <bag>            (per bag)
#   python3 evaluation/check_dual_sync.py --bag-a A --bag-b B    (cross-Jetson)
set -euo pipefail

ROLE="${1:?usage: setup_time_sync.sh <master|slave> [IFACE] [MASTER_IP]}"
IFACE="${2:-eth0}"
MASTER_IP="${3:-}"
[ "$(id -u)" -eq 0 ] || { echo "run with sudo"; exit 1; }
case "$ROLE" in master|slave) ;; *) echo "role must be master|slave"; exit 1;; esac
if [ "$ROLE" = slave ] && [ -z "$MASTER_IP" ]; then
    echo "slave role needs MASTER_IP (the camera Jetson's switch IP)"; exit 1
fi

echo "[1/4] installing chrony + linuxptp + ethtool"
apt-get update -qq
apt-get install -y chrony linuxptp ethtool

# Hardware vs software timestamping decides whether we need phc2sys.
if ethtool -T "$IFACE" | grep -q "hardware-transmit"; then
    HW=1; echo "  ${IFACE}: hardware timestamping available"
else
    HW=0; echo "  ${IFACE}: software timestamping only (fine at the ms level)"
fi

echo "[2/4] /etc/chrony/chrony.conf (backup kept as .orig)"
[ -f /etc/chrony/chrony.conf.orig ] || cp /etc/chrony/chrony.conf /etc/chrony/chrony.conf.orig
if [ "$ROLE" = master ]; then
cat > /etc/chrony/chrony.conf <<'EOF'
# MMB CAMERA/GNSS Jetson — PTP grandmaster + LAN NTP server.
pool pool.ntp.org iburst maxsources 4
# Field use: keep serving a consistent (if unreferenced) clock when offline.
local stratum 10
allow 192.168.0.0/16
allow 10.0.0.0/8
driftfile /var/lib/chrony/chrony.drift
makestep 1.0 3
rtcsync
# GNSS UPGRADE (outdoors): add a PPS/NMEA refclock here to make the grandmaster
# UTC-disciplined, e.g.:
#   refclock SHM 0 refid NMEA offset 0.0 precision 1e-1
#   refclock PPS /dev/pps0 lock NMEA refid PPS
EOF
else
cat > /etc/chrony/chrony.conf <<EOF
# MMB LiDAR Jetson — PTP owns the system clock (phc2sys). chrony only provides
# a sane fallback from the master if PTP drops; it must NOT fight phc2sys, so no
# local stratum / no serving.
server ${MASTER_IP} iburst minpoll 4 maxpoll 4
driftfile /var/lib/chrony/chrony.drift
makestep 1.0 3
rtcsync
EOF
fi
systemctl enable --now chrony
systemctl restart chrony

echo "[3/4] ptp4l (${ROLE}) on ${IFACE}"
if [ "$ROLE" = master ]; then
    PTP_ARGS="--masterOnly 1 --priority1 10"
    DESC="PTP grandmaster for the rig (${IFACE})"
else
    PTP_ARGS="--slaveOnly 1"
    DESC="PTP slave -> grandmaster (${IFACE})"
fi
[ "$HW" -eq 1 ] || PTP_ARGS="$PTP_ARGS -S"

cat > /etc/systemd/system/mmb-ptp4l.service <<EOF
[Unit]
Description=${DESC}
After=network-online.target
Wants=network-online.target

[Service]
ExecStart=/usr/sbin/ptp4l -i ${IFACE} ${PTP_ARGS} -m
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

# phc2sys (only with hardware timestamping; -S serves/uses CLOCK_REALTIME):
#   master: system clock -> NIC PHC, so PTP-on-wire == system time.
#   slave : NIC PHC (disciplined by ptp4l) -> system clock.
if [ "$HW" -eq 1 ]; then
    if [ "$ROLE" = master ]; then
        PHC2SYS_EXEC="/usr/sbin/phc2sys -c ${IFACE} -s CLOCK_REALTIME -O 0 -m"
    else
        PHC2SYS_EXEC="/usr/sbin/phc2sys -s ${IFACE} -O 0 -m"   # PHC -> system
    fi
cat > /etc/systemd/system/mmb-phc2sys.service <<EOF
[Unit]
Description=phc2sys (${ROLE}) for ${IFACE}
After=mmb-ptp4l.service
Requires=mmb-ptp4l.service

[Service]
ExecStart=${PHC2SYS_EXEC}
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
fi

systemctl daemon-reload
systemctl enable --now mmb-ptp4l
[ "$HW" -eq 1 ] && systemctl enable --now mmb-phc2sys

echo "[4/4] status"
chronyc tracking 2>/dev/null | sed 's/^/  chrony: /' || true
systemctl --no-pager -l status mmb-ptp4l | head -5 | sed 's/^/  /'
echo
if [ "$ROLE" = master ]; then
cat <<'EOF'
Done (master). Remaining manual steps:
 1. Hesai JT128: web UI (http://192.168.1.201) -> Network -> PTP -> slave;
    wait ~30 s, status should read "Locked".
 2. Livox Avia: nothing — it picks up the PTP master automatically.
 3. Run `setup_time_sync.sh slave <iface> <THIS_JETSON_IP>` on the LiDAR Jetson.
EOF
else
cat <<EOF
Done (slave). Watch it lock to the grandmaster:
  journalctl -fu mmb-ptp4l        # "rms"/"offset" should settle (< ~1 us HW, ms SW)
  chronyc sources                 # ${MASTER_IP} reachable as a fallback
EOF
fi
