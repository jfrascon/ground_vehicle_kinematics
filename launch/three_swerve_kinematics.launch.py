from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity  # noqa
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory

import ros2_launch_helpers as rlh

import os


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('namespace', default_value='robot', description='Namepace'),
            DeclareLaunchArgument('robot_prefix', default_value='robot_', description="Robot's prefix"),
            DeclareLaunchArgument(
                'params_file',
                default_value=os.path.join(
                    get_package_share_directory('ground_vehicle_kinematics'),
                    'config',
                    'example_three_swerve_kinematics.yaml',
                ),
                description='Base YAML with ros__parameters',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='False',
                choices=['True', 'true', 'False', 'false'],
                description='Use simulation clock if true',
            ),
            DeclareLaunchArgument('node_name', default_value='three_swerve_kinematics', description='Node name'),
            DeclareLaunchArgument('node_remappings', default_value='{}', description=rlh.REMAPPINGS_DESC),
            DeclareLaunchArgument('node_logging_options', default_value='{}', description=rlh.LOGGING_OPTIONS_DESC),
            DeclareLaunchArgument('node_options', default_value='{}', description=rlh.NODE_OPTIONS_DESC),
            OpaqueFunction(function=launch_kinematics_node),
        ]
    )


def launch_kinematics_node(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    parameters = []
    params_file = LaunchConfiguration('params_file').perform(ctx).strip()

    # Add parameter file only if it's not empty.
    if params_file:
        parameters.append(ParameterFile(params_file, allow_substs=True))

    parameters.append({'use_sim_time': LaunchConfiguration('use_sim_time')})

    node_name = LaunchConfiguration('node_name').perform(ctx)
    node_options_by_name, remappings_by_name, ros_arguments_by_name = rlh.resolve_node_launch_configs(
        node_names=[node_name],
        node_options=LaunchConfiguration('node_options').perform(ctx),
        node_logging_options=LaunchConfiguration('node_logging_options').perform(ctx),
        node_remappings=LaunchConfiguration('node_remappings').perform(ctx),
    )
    node_options = node_options_by_name[node_name]
    remappings = remappings_by_name[node_name]
    ros_arguments = ros_arguments_by_name[node_name]

    return [
        Node(
            package='ground_vehicle_kinematics',
            executable='three_swerve_kinematics_node',
            name=node_name,
            namespace=LaunchConfiguration('namespace'),
            parameters=parameters,
            remappings=remappings,
            ros_arguments=ros_arguments,
            output=node_options['output'],
            emulate_tty=node_options['emulate_tty'],
            respawn=node_options['respawn'],
            respawn_delay=node_options['respawn_delay'],
        )
    ]
