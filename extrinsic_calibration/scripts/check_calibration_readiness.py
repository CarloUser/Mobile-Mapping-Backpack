#!/usr/bin/env python3
"""Check that the extrinsic-calibration repo setup is ready to run.

This is an offline sanity check. It validates YAML structure, confirmed LiDAR
topics, static TF frames, and Python syntax without requiring ROS to be sourced.
Use --strict-deps on the PC that will run LiDAR-LiDAR calibration if you also
want missing Python/CLI dependencies to fail the check.
"""
import argparse
import importlib.util
import math
import shutil
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]

EXTRINSICS = REPO_ROOT / "ros2_ws" / "src" / "mmb_bringup" / "config" / "extrinsics_initial.yaml"
TOPICS = REPO_ROOT / "ros2_ws" / "src" / "mmb_bringup" / "config" / "topics.yaml"
LIDAR_LIDAR_CONFIG = REPO_ROOT / "extrinsic_calibration" / "lidar_lidar" / "config.yaml"
SETUP_PY = REPO_ROOT / "ros2_ws" / "src" / "mmb_bringup" / "setup.py"

REQUIRED_CHILD_FRAMES = {
    "lidar_hesai",
    "lidar_livox_avia",
    "camera_oak1",
    "camera_oak4d",
    "camera_oakd_lite",
    "camera_realsense",
    "camera_insta360",
    "imu_xsens",
    "gnss_antenna",
}

REQUIRED_RECORDING_TOPICS = {
    "/lidar_points",
    "/livox/lidar",
    "/livox/imu",
    "/tf",
    "/tf_static",
}

COMPILE_TARGETS = [
    REPO_ROOT / "ros2_ws" / "src" / "mmb_bringup" / "launch" / "all_sensors.launch.py",
    REPO_ROOT / "ros2_ws" / "src" / "mmb_bringup" / "launch" / "lidar.launch.py",
    REPO_ROOT / "ros2_ws" / "src" / "mmb_bringup" / "launch" / "recording.launch.py",
    REPO_ROOT / "ros2_ws" / "src" / "mmb_bringup" / "launch" / "static_extrinsics.launch.py",
    REPO_ROOT / "extrinsic_calibration" / "scripts" / "validate_initial_extrinsics.py",
    REPO_ROOT / "extrinsic_calibration" / "lidar_lidar" / "handeye.py",
    REPO_ROOT / "extrinsic_calibration" / "lidar_lidar" / "io_utils.py",
    REPO_ROOT / "extrinsic_calibration" / "lidar_lidar" / "run_odometry.py",
    REPO_ROOT / "extrinsic_calibration" / "lidar_lidar" / "solve_extrinsic.py",
    REPO_ROOT / "extrinsic_calibration" / "lidar_lidar" / "validate.py",
    REPO_ROOT / "extrinsic_calibration" / "lidar_lidar" / "test_handeye.py",
]


def load_yaml(path):
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def rel(path):
    return path.relative_to(REPO_ROOT)


def quaternion_norm(q_xyzw):
    return math.sqrt(sum(float(value) * float(value) for value in q_xyzw))


def record_pass(checks, message):
    checks.append(("PASS", message))


def record_warning(checks, message):
    checks.append(("WARN", message))


def record_failure(checks, message):
    checks.append(("FAIL", message))


def validate_extrinsics(checks):
    doc = load_yaml(EXTRINSICS)
    reference_frame = doc.get("reference_frame", "base_link")
    transforms = doc.get("transforms", [])
    seen = set()

    if reference_frame != "base_link":
        record_failure(checks, f"{rel(EXTRINSICS)} reference_frame is {reference_frame}, expected base_link")

    for transform in transforms:
        child = transform.get("child")
        parent = transform.get("parent")
        xyz = transform.get("xyz", [])
        q_xyzw = transform.get("q_xyzw", [])

        if child in seen:
            record_failure(checks, f"duplicate child frame in extrinsics: {child}")
        seen.add(child)

        if parent != reference_frame:
            record_failure(checks, f"{child}: parent {parent} does not match {reference_frame}")
        if len(xyz) != 3:
            record_failure(checks, f"{child}: xyz must contain 3 values")
        if len(q_xyzw) != 4:
            record_failure(checks, f"{child}: q_xyzw must contain 4 values")
        elif abs(quaternion_norm(q_xyzw) - 1.0) > 1e-6:
            record_failure(checks, f"{child}: quaternion is not normalized")

    missing = sorted(REQUIRED_CHILD_FRAMES - seen)
    if missing:
        record_failure(checks, "missing expected frames: " + ", ".join(missing))
    else:
        record_pass(checks, f"{rel(EXTRINSICS)} has all {len(REQUIRED_CHILD_FRAMES)} expected frames")


def validate_topics(checks):
    topics_doc = load_yaml(TOPICS)
    topics = set(topics_doc.get("topics", []))
    missing = sorted(REQUIRED_RECORDING_TOPICS - topics)
    if missing:
        record_failure(checks, f"{rel(TOPICS)} missing topics: " + ", ".join(missing))
    else:
        record_pass(checks, f"{rel(TOPICS)} includes confirmed LiDAR and TF topics")

    cfg = load_yaml(LIDAR_LIDAR_CONFIG)
    hesai_topic = cfg["bag"].get("hesai_topic")
    livox_topic = cfg["bag"].get("livox_topic")
    if hesai_topic != "/lidar_points":
        record_failure(checks, f"LiDAR-LiDAR Hesai topic is {hesai_topic}, expected /lidar_points")
    if livox_topic != "/livox/lidar":
        record_failure(checks, f"LiDAR-LiDAR Livox topic is {livox_topic}, expected /livox/lidar")
    if hesai_topic == "/lidar_points" and livox_topic == "/livox/lidar":
        record_pass(checks, f"{rel(LIDAR_LIDAR_CONFIG)} uses confirmed Hesai/Livox topics")

    initial = (LIDAR_LIDAR_CONFIG.parent / cfg["paths"]["initial_extrinsics"]).resolve()
    if not initial.exists():
        record_failure(checks, f"initial_extrinsics path does not exist: {initial}")
    else:
        record_pass(checks, "LiDAR-LiDAR initial extrinsics path resolves")


def validate_packaging(checks):
    text = SETUP_PY.read_text(encoding="utf-8")
    required_snippets = ["glob('launch/*.launch.py')", "glob('config/*.yaml')"]
    missing = [snippet for snippet in required_snippets if snippet not in text]
    if missing:
        record_failure(checks, f"{rel(SETUP_PY)} is missing install globs: " + ", ".join(missing))
    else:
        record_pass(checks, "mmb_bringup installs launch files and config YAMLs")


def validate_syntax(checks):
    for path in COMPILE_TARGETS:
        compile(path.read_text(encoding="utf-8"), str(path), "exec")
    record_pass(checks, f"compiled {len(COMPILE_TARGETS)} launch/calibration scripts")


def validate_dependencies(checks, strict_deps):
    deps = {
        "numpy": "numpy",
        "scipy": "scipy",
        "yaml": "pyyaml",
        "matplotlib": "matplotlib",
    }
    missing = [package for module, package in deps.items() if importlib.util.find_spec(module) is None]
    if shutil.which("kiss_icp_pipeline") is None:
        missing.append("kiss-icp CLI: kiss_icp_pipeline")

    if not missing:
        record_pass(checks, "PC LiDAR-LiDAR Python/CLI dependencies are available")
    elif strict_deps:
        record_failure(checks, "missing PC LiDAR-LiDAR dependencies: " + ", ".join(missing))
    else:
        record_warning(
            checks,
            "missing PC LiDAR-LiDAR dependencies ("
            + ", ".join(missing)
            + "); install before running odometry: "
            "pip install -r extrinsic_calibration/lidar_lidar/requirements.txt",
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--strict-deps",
        action="store_true",
        help="Fail if PC-side LiDAR-LiDAR Python/CLI dependencies are missing.",
    )
    args = parser.parse_args()

    checks = []
    validate_extrinsics(checks)
    validate_topics(checks)
    validate_packaging(checks)
    validate_syntax(checks)
    validate_dependencies(checks, args.strict_deps)

    for status, message in checks:
        print(f"[{status}] {message}")

    failures = [message for status, message in checks if status == "FAIL"]
    if failures:
        print(f"\nReadiness check failed with {len(failures)} issue(s).")
        raise SystemExit(1)

    warnings = [message for status, message in checks if status == "WARN"]
    if warnings:
        print(f"\nReadiness check passed with {len(warnings)} warning(s).")
    else:
        print("\nReadiness check passed.")


if __name__ == "__main__":
    main()
