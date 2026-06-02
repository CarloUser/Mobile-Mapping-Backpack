#!/usr/bin/env python3
import argparse
import math
from pathlib import Path

import yaml


REQUIRED_CHILD_FRAMES = {
    'lidar_hesai',
    'lidar_livox_avia',
    'camera_oak1',
    'camera_oak4d',
    'camera_oakd_lite',
    'camera_realsense',
    'camera_insta360',
    'imu_xsens',
    'gnss_antenna',
}


def default_config_path():
    repo_root = Path(__file__).resolve().parents[2]
    return repo_root / 'ros2_ws' / 'src' / 'mmb_bringup' / 'config' / 'extrinsics_initial.yaml'


def quaternion_norm(q_xyzw):
    return math.sqrt(sum(float(value) * float(value) for value in q_xyzw))


def main():
    parser = argparse.ArgumentParser(
        description='Validate the CAD-derived initial extrinsics YAML.'
    )
    parser.add_argument(
        'config',
        nargs='?',
        type=Path,
        default=default_config_path(),
        help='Path to extrinsics_initial.yaml.',
    )
    args = parser.parse_args()

    with args.config.open('r', encoding='utf-8') as stream:
        config = yaml.safe_load(stream)

    transforms = config.get('transforms', [])
    seen = set()
    errors = []

    for transform in transforms:
        child = transform.get('child')
        parent = transform.get('parent')
        xyz = transform.get('xyz', [])
        q_xyzw = transform.get('q_xyzw', [])

        if child in seen:
            errors.append(f'duplicate child frame: {child}')
        seen.add(child)

        if parent != config.get('reference_frame', 'base_link'):
            errors.append(f'{child}: parent {parent} does not match reference_frame')
        if len(xyz) != 3:
            errors.append(f'{child}: xyz must contain 3 values')
        if len(q_xyzw) != 4:
            errors.append(f'{child}: q_xyzw must contain 4 values')
            continue

        norm = quaternion_norm(q_xyzw)
        if abs(norm - 1.0) > 1e-6:
            errors.append(f'{child}: quaternion norm is {norm:.9f}, expected 1.0')

    missing = sorted(REQUIRED_CHILD_FRAMES - seen)
    if missing:
        errors.append('missing expected frames: ' + ', '.join(missing))

    print(f'Config: {args.config}')
    print(f'Transforms: {len(transforms)}')
    for transform in transforms:
        args_text = [
            *[str(value) for value in transform['xyz']],
            *[str(value) for value in transform['q_xyzw']],
            transform['parent'],
            transform['child'],
        ]
        print('  static_transform_publisher ' + ' '.join(args_text))

    if errors:
        print('\nValidation errors:')
        for error in errors:
            print(f'  - {error}')
        raise SystemExit(1)

    print('\nValidation passed.')


if __name__ == '__main__':
    main()
