#!/usr/bin/env python3
"""One-shot: re-express the rig extrinsics from the SolidWorks CAD convention
(base_link = X-forward, Y-up, Z-right) into ROS REP-103 (X-forward, Y-left,
Z-up). base_link's orientation is arbitrary, so we simply redefine it as REP-103.

Global reorientation:  R_CAD2ROS = Rx(+90deg) = [[1,0,0],[0,0,-1],[0,1,0]]
  v_ros = R_CAD2ROS @ v_cad   (up: +Y_cad -> +Z_ros ; right: +Z_cad -> -Y_ros)

For every transform T_base_child:
  p_ros = R_CAD2ROS @ p_cad
  R_ros = R_CAD2ROS @ R_cad

CAMERAS additionally switch from the SolidWorks part frame (X-fwd/Y-up/Z-right)
to the ROS camera OPTICAL convention (X-right/Y-down/Z-fwd), so camera_info /
images / cv2 poses line up with no fudge matrix in the calibrator:
  R_ros_optical = R_CAD2ROS @ R_cad @ R_OPT2CAD
where R_OPT2CAD = [[0,0,1],[0,-1,0],[1,0,0]] is part<-optical (cad X=opt Z,
Y=-opt Y, Z=opt X). VERIFIED for the OAK-4D (upright/forward). For the other
cameras this assumes the SolidWorks part frames share that convention; it only
seeds the LiDAR crop, so a small error there is harmless and each camera's own
solve still recovers the true optical pose.

Source of truth for the original CAD values stays Coordinate_systems/
coordinate_systems.yaml (NOT modified).

Usage: python3 convert_cad_to_ros_frame.py <in.yaml> <out.yaml>
"""
import sys
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

R_CAD2ROS = np.array([[1.0, 0, 0], [0, 0, -1.0], [0, 1.0, 0]])   # Rx(+90)
R_OPT2CAD = np.array([[0, 0, 1.0], [0, -1.0, 0], [1.0, 0, 0]])   # part <- optical

CAMERA_PREFIX = "camera_"


def convert_entry(tf):
    q = tf["q_xyzw"]
    p = np.array(tf["xyz"], float)
    Rcad = R.from_quat(q).as_matrix()
    p_ros = R_CAD2ROS @ p
    if str(tf["child"]).startswith(CAMERA_PREFIX):
        R_ros = R_CAD2ROS @ Rcad @ R_OPT2CAD          # -> optical frame
        frame_kind = "optical"
    else:
        R_ros = R_CAD2ROS @ Rcad                       # -> body frame, re-referenced
        frame_kind = "body"
    q_ros = R.from_matrix(R_ros).as_quat()
    return p_ros, q_ros, frame_kind


def main():
    src, dst = Path(sys.argv[1]), Path(sys.argv[2])
    doc = yaml.safe_load(open(src))
    print(f"[convert] {src.name}: base_link CAD(X-fwd,Y-up,Z-right) -> "
          f"REP-103(X-fwd,Y-left,Z-up)")
    for tf in doc["transforms"]:
        p_ros, q_ros, kind = convert_entry(tf)
        old_q = np.round(tf["q_xyzw"], 4).tolist()
        tf["xyz"] = [float(round(v, 12)) for v in p_ros]
        tf["q_xyzw"] = [float(round(v, 12)) for v in q_ros]
        print(f"  {tf['child']:18s} [{kind:7s}] q {str(old_q):>34} -> "
              f"{np.round(q_ros,4).tolist()}")
    # provenance / header note
    note = ("Re-expressed from the SolidWorks CAD convention (X-fwd,Y-up,Z-right) "
            "into ROS REP-103 (X-fwd,Y-left,Z-up) by Rx(+90deg); camera_* frames "
            "are ROS OPTICAL frames. Source of truth: "
            "Coordinate_systems/coordinate_systems.yaml.")
    if isinstance(doc.get("provenance"), list):
        doc["provenance"].append({"stage": "cad_to_ros_frame", "note": note})
    body = yaml.safe_dump(doc, sort_keys=False, default_flow_style=None)
    with open(dst, "w") as f:
        f.write(f"# {note}\n\n")
        f.write(body)
    print(f"[convert] wrote {dst}")


if __name__ == "__main__":
    main()
