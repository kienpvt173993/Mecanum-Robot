import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
	# Get the launch directory
    mecanum_gazebo_pkg_dir = get_package_share_directory('mecanum_robot_gazebo')
    mecanum_ros_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mecanum_gazebo_pkg_dir, 'launch', 'mecanum_ros_control.launch.py')),
        launch_arguments={'use_sim_time': 'True'}.items(),
    )
    mecanum_robot_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mecanum_gazebo_pkg_dir, 'launch', 'mecanum_robot_gazebo.launch.py')),
        launch_arguments={'use_sim_time': 'True'}.items(),
    )
    return LaunchDescription([
        mecanum_ros_control,
        mecanum_robot_gazebo
    ])