import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_config_dir = os.path.join(
        get_package_share_directory('mecanum_robot_description'),
        'rviz',
        'robot_demo.rviz'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='use_sim_time')

    robot_description_pkg_dir = os.path.join(get_package_share_directory('mecanum_robot_description'), 'launch')
    
    launch_file_name = 'mecanum_robot_description.launch.py'

    return LaunchDescription([
        declare_use_sim_time_cmd,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[
            {
                "use_sim_time": use_sim_time
            }],
            output='screen'),

        Node(package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[
            {
                "use_sim_time": use_sim_time
            }],
            output='screen'),

        Node(package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            parameters=[
            {
                "use_sim_time": use_sim_time
            }],
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(robot_description_pkg_dir, launch_file_name)),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])