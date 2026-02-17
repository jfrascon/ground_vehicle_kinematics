from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity  # noqa
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory

import ros2_launch_helpers as rlh

import os


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='False',
                choices=['True', 'true', 'False', 'false'],
                description='Use simulation clock if true',
            ),
            DeclareLaunchArgument('namespace', default_value='', description='namespace (Optional, default: "")'),
            DeclareLaunchArgument('robot_name', default_value='robot', description='The unique name for the robot'),
            DeclareLaunchArgument(
                'params_file',
                default_value=os.path.join(
                    get_package_share_directory('ground_vehicle_kinematics'),
                    'config',
                    'example_three_swerve_kinematics.yaml',
                ),
                description='Base YAML with ros__parameters',
            ),
            DeclareLaunchArgument('topic_remappings', default_value='', description=rlh.TOPIC_REMAPPINGS_DESC),
            DeclareLaunchArgument(
                'logging_options', default_value=rlh.default_logging_options_str(), description=rlh.LOGGING_OPTIONS_DESC
            ),
            DeclareLaunchArgument(
                'node_options', default_value=rlh.default_node_options_str(), description=rlh.NODE_OPTIONS_DESC
            ),
            OpaqueFunction(function=launch_three_swerve_kinematics_node),
        ]
    )


def launch_three_swerve_kinematics_node(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    parameters = []
    params_file = LaunchConfiguration('params_file').perform(ctx).strip()

    # Add parameter file only if it's not empty.
    if params_file:
        parameters.append(ParameterFile(params_file, allow_substs=True))

    parameters.append({'use_sim_time': LaunchConfiguration('use_sim_time')})

    node_options = rlh.process_node_options(LaunchConfiguration('node_options').perform(ctx))
    node_name = str(node_options['name']) or 'three_swerve_kinematics'

    if not rlh.is_valid_name(node_name):
        raise RuntimeError(f"The name of the node must be ASCII [A-Za-z0-9_] only: '{node_name}'")

    robot_name = LaunchConfiguration('robot_name').perform(ctx)
    robot_ns = rlh.create_robot_namespace(LaunchConfiguration('namespace').perform(ctx), robot_name)

    return [
        # The parameter file uses variable expansion for robot_prefix, so we need to set the parameter 'robot_prefix'
        # before adding the Node action that uses that parameter file.
        SetLaunchConfiguration('robot_prefix', rlh.create_robot_prefix(robot_name)),
        Node(
            package='ground_vehicle_kinematics',
            executable='three_swerve_kinematics_node',
            name=node_name,
            namespace=robot_ns,
            parameters=parameters,
            remappings=rlh.process_topic_remappings(LaunchConfiguration('topic_remappings').perform(ctx)),
            ros_arguments=rlh.process_logging_options(LaunchConfiguration('logging_options').perform(ctx)),
            output=node_options['output'],
            emulate_tty=node_options['emulate_tty'],
            respawn=node_options['respawn'],
            respawn_delay=node_options['respawn_delay'],
        ),
    ]
