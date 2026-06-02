from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def _as_cli(value):
    return f"{float(value):.16g}"


def _create_static_tf_nodes(context, *args, **kwargs):
    extrinsics_file = LaunchConfiguration('extrinsics_file').perform(context)
    with open(extrinsics_file, 'r', encoding='utf-8') as stream:
        config = yaml.safe_load(stream)

    nodes = []
    for transform in config.get('transforms', []):
        parent = transform['parent']
        child = transform['child']
        xyz = transform['xyz']
        q_xyzw = transform['q_xyzw']
        safe_child = child.replace('/', '_').replace('-', '_')

        nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'tf_base_to_{safe_child}',
                arguments=[
                    *[_as_cli(value) for value in xyz],
                    *[_as_cli(value) for value in q_xyzw],
                    parent,
                    child,
                ],
            )
        )
    return nodes


def generate_launch_description():
    pkg = get_package_share_directory('mmb_bringup')
    default_extrinsics_file = os.path.join(
        pkg, 'config', 'extrinsics_initial.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'extrinsics_file',
            default_value=default_extrinsics_file,
            description='YAML file containing static sensor extrinsics.',
        ),
        OpaqueFunction(function=_create_static_tf_nodes),
    ])
