from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.utilities.type_utils import normalize_typed_substitution, perform_typed_substitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile, ParameterValue
import ros2_launch_helpers as rlh


def generate_launch_description() -> LaunchDescription:
    """Declare the inputs required to launch the four-swerve kinematics node."""
    return LaunchDescription(
        [
            DeclareLaunchArgument('namespace', default_value='', description='Node namespace.'),
            DeclareLaunchArgument(
                'robot_prefix', default_value='', description='Prefix used in joint names.'
            ),
            DeclareLaunchArgument(
                'params_file',
                default_value=PathJoinSubstitution(
                    [
                        get_package_share_directory('ground_vehicle_kinematics'),
                        'config',
                        'example_four_swerve_kinematics.yaml',
                    ]
                ),
                description='Path to the complete node parameter YAML file.',
            ),
            DeclareLaunchArgument(
                'params_file_allow_substs',
                default_value='True',
                choices=['True', 'true', 'False', 'false'],
                description='Allow ROS launch substitutions in params_file.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='False',
                choices=['True', 'true', 'False', 'false'],
                description='Use ROS simulation time when true.',
            ),
            DeclareLaunchArgument(
                'node_args',
                default_value='{"output":"both","ros_arguments":["--log-level","info"]}',
                description=rlh.LAUNCH_ACTION_ARGUMENTS_DESC,
            ),
            rlh.RequireFile(path=LaunchConfiguration('params_file')),
            OpaqueFunction(function=_launch_node),
        ]
    )


def _launch_node(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    """Launch the node after resolving whether its parameter file allows substitutions."""
    allow_substs = perform_typed_substitution(
        ctx,
        normalize_typed_substitution(LaunchConfiguration('params_file_allow_substs'), bool),
        bool,
    )

    return [
        Node(
            package='ground_vehicle_kinematics',
            executable='four_swerve_kinematics_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                ParameterFile(LaunchConfiguration('params_file'), allow_substs=allow_substs),
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool
                    )
                },
            ],
            **rlh.resolve_node_arguments(
                LaunchConfiguration('node_args').perform(ctx),
                default_arguments={'name': 'four_swerve_kinematics'},
                extra_rejected_arguments={'namespace'},
            ),
        )
    ]
