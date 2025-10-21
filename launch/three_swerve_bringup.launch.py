from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity  # noqa
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameters_type import SomeParameters  # noqa
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory

import ros_launch_helpers as rlh

import os


def generate_launch_description():
    pkg_share = get_package_share_directory('ground_vehicle_kinematics')
    default_params_file = os.path.join(pkg_share, 'config', 'example_three_swerve_kinematics.yaml')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='False',
                choices=['True', 'true', 'False', 'false'],
                description='Use simulation clock if true',
            ),
            DeclareLaunchArgument('namespace', default_value='', description='ROS namespace (optional, default: "")'),
            DeclareLaunchArgument('robot_name', default_value='robot', description='The unique name for the robot'),
            DeclareLaunchArgument('node_name', default_value='three_swerve_kinematics', description='Node name'),
            DeclareLaunchArgument(
                'params_file', default_value=default_params_file, description='Base YAML with ros__parameters'
            ),
            DeclareLaunchArgument(
                'overlay_params_file_list',
                default_value='',
                description='Comma-separated list of YAML overlays (applied in order; last wins).',
            ),
            DeclareLaunchArgument('remappings', default_value='', description=rlh.REMAPPINGS_DESC),
            DeclareLaunchArgument(
                'log_options', default_value=rlh.default_log_options_str(), description=rlh.LOG_OPTIONS_DESC
            ),
            DeclareLaunchArgument(
                'node_options', default_value=rlh.default_node_options_str(), description=rlh.NODE_OPTIONS_DESC
            ),
            OpaqueFunction(function=rlh.set_robot_namespace, args=['namespace', 'robot_name']),
            OpaqueFunction(function=rlh.set_robot_prefix, args=['robot_name']),
            OpaqueFunction(function=launch_ground_kinematics_node),
        ]
    )


def launch_ground_kinematics_node(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    parameters = rlh.get_params(
        LaunchConfiguration('params_file').perform(ctx), LaunchConfiguration('overlay_params_file_list').perform(ctx)
    )

    # Add to the parameters the robot prefix and the use_sim_time flag.
    parameters.append(
        {'robot_prefix': LaunchConfiguration('robot_prefix'), 'use_sim_time': LaunchConfiguration('use_sim_time')}
    )

    node_options = rlh.parse_cli_node_opts(LaunchConfiguration('node_options').perform(ctx))

    node = LifecycleNode(
        package='ground_vehicle_kinematics',
        executable='three_swerve_kinematics_node',
        name=LaunchConfiguration('node_name').perform(ctx),
        namespace=LaunchConfiguration('robot_namespace').perform(ctx),
        parameters=parameters,
        remappings=rlh.parse_cli_remappings(LaunchConfiguration('remappings').perform(ctx)),
        ros_arguments=rlh.parse_cli_log_opts(LaunchConfiguration('log_options').perform(ctx)),
        output=node_options['output'],
        emulate_tty=node_options['emulate_tty'],
        respawn=node_options['respawn'],
        respawn_delay=node_options['respawn_delay'],
    )

    configure = EmitEvent(
        event=ChangeState(lifecycle_node_matcher=lambda n: n == node, transition_id=Transition.TRANSITION_CONFIGURE)
    )

    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda n: n == node, transition_id=Transition.TRANSITION_ACTIVATE
                    )
                )
            ],
        )
    )

    return [node, configure, activate]
