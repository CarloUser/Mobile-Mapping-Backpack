#!/usr/bin/env bash
# Time-sync stack for the mapping rig (run with sudo on the Jetson):
#   - chrony  : NTP for the Jetson system clock (internet when available,
#               free-running fallback in the field, serves NTP to the LAN)
#   - linuxptp: ptp4l PTP master on the LiDAR Ethernet so BOTH LiDARs
#               (Hesai JT128 + Livox Avia, IEEE 1588v2 slaves) discipline
#               their internal clocks to the Jetson.
#
# Usage:  sudo bash setup_time_sync.sh [LIDAR_IFACE]    (default: eth0)
#
# After running: enable PTP slave mode in the Hesai web UI
# (http://192.168.1.201 -> Network -> PTP). The Livox follows a PTP master
# automatically (sync priority PTP > GPS > PPS).
#
# Verify a recording afterwards with:
#   python3 evaluation/check_time_sync.py --bag <bag>
set -euo pipefail

IFACE="${1:-eth0}"
[ "$(id -u)" -eq 0 ] || { echo "run with sudo"; exit 1; }

echo "[1/4] installing chrony + linuxptp + ethtool"
apt-get update -qq
apt-get install -y chrony linuxptp ethtool

echo "[2/4] writing /etc/chrony/chrony.conf (backup kept as .orig)"
[ -f /etc/chrony/chrony.conf.orig ] || cp /etc/chrony/chrony.conf /etc/chrony/chrony.conf.orig
cat > /etc/chrony/chrony.conf <<'EOF'
# Mobile Mapping Backpack — Jetson time config (deployed by setup_time_sync.sh)
pool pool.ntp.org iburst maxsources 4

# Field use: keep serving a consistent (if unreferenced) clock when offline.
local stratum 10

# Serve NTP to anything on the rig LANs.
allow 192.168.0.0/16
allow 10.0.0.0/8

driftfile /var/lib/chrony/chrony.drift
makestep 1.0 3
rtcsync
EOF
systemctl enable --now chrony
systemctl restart chrony

echo "[3/4] PTP master on ${IFACE}"
if ethtool -T "$IFACE" | grep -q "hardware-transmit"; then
    TSTAMP_ARGS=""
    echo "  ${IFACE}: hardware timestamping available"
else
    TSTAMP_ARGS="-S"
    echo "  ${IFACE}: software timestamping only (fine at the ms level)"
fi

cat > /etc/systemd/system/mmb-ptp4l.service <<EOF
[Unit]
Description=PTP master for rig LiDARs (${IFACE})
After=network-online.target
Wants=network-online.target

[Service]
ExecStart=/usr/sbin/ptp4l -i ${IFACE} ${TSTAMP_ARGS} --masterOnly 1 --priority1 10
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

# With hardware timestamping the NIC PHC must follow the (chrony-disciplined)
# system clock so PTP time on the wire == system time. Skip for software mode
# (ptp4l -S serves CLOCK_REALTIME directly).
if [ -z "$TSTAMP_ARGS" ]; then
cat > /etc/systemd/system/mmb-phc2sys.service <<EOF
[Unit]
Description=Sync ${IFACE} PHC from system clock for PTP master
After=mmb-ptp4l.service
Requires=mmb-ptp4l.service

[Service]
ExecStart=/usr/sbin/phc2sys -c ${IFACE} -s CLOCK_REALTIME -O 0 -m
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
fi

systemctl daemon-reload
systemctl enable --now mmb-ptp4l
[ -z "$TSTAMP_ARGS" ] && systemctl enable --now mmb-phc2sys

echo "[4/4] status"
chronyc tracking | sed 's/^/  chrony: /' || true
systemctl --no-pager -l status mmb-ptp4l | head -5 | sed 's/^/  /'
cat <<'EOF'

Done. Remaining manual steps:
 1. Hesai JT128: web UI (http://192.168.1.201) -> Network -> PTP -> slave;
    wait ~30 s, status should read "Locked".
 2. Livox Avia: nothing — it picks up the PTP master automatically.
 3. Verify live:  ros2 topic echo /lidar_points --field header.stamp --once
    and compare against `date +%s` — should agree to ~ms.
 4. Verify recorded bags: python3 evaluation/check_time_sync.py --bag <bag>
EOF
