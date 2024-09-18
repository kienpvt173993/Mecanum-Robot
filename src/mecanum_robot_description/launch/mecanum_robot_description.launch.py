import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='use_sim_time')
    xacro_file = os.path.join(get_package_share_directory('mecanum_robot_description'), 'urdf/', 'zm_robot.urdf.xacro')    
    assert os.path.exists(xacro_file), "The box_bot.xacro doesn't exist in "+ str(xacro_file)

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    return LaunchDescription([
        declare_use_sim_time_cmd,
           
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {
                    "robot_description": robot_desc,
                    "use_sim_time": use_sim_time
                }],
            output="screen")
    ])