from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use gazebo time if True'
    )
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("mecanum_robot_gazebo"),
            "config",
            "mecanum_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers,
        {
            'use_sim_time': use_sim_time
        }],
        output="both",
        remappings=[
            ("/mecanum_bot_base_controller/cmd_vel", "/cmd_vel"),
            ("/mecanum_bot_base_controller/cmd_vel_unstamped", "/cmd_vel"),
            ("/controller_manager/robot_description", "/robot_description")
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        executable="spawner",
        arguments=["mecanum_bot_base_controller", "--param-file", robot_controllers],
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    nodes = [
        declare_use_sim_time,
        # control_node,
        robot_controller_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
    ]

    return LaunchDescription(nodes)