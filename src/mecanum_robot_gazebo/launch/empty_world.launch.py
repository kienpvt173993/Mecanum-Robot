import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
	# Get the launch directory
	mecanum_gazebo_pkg_dir = get_package_share_directory('mecanum_robot_gazebo')

	# Launch configuration variables specific to simulation
	headless = LaunchConfiguration('headless')
	world = LaunchConfiguration('world')

	declare_use_sim_time_cmd = DeclareLaunchArgument(
		'use_sim_time',
		default_value='True',
		description='Use simulation (Gazebo) clock if true')

	declare_simulator_cmd = DeclareLaunchArgument(
		'headless',
		default_value='False',
		description='Whether to execute gzclient)')

	declare_world_cmd = DeclareLaunchArgument(
		'world',
		default_value=os.path.join(mecanum_gazebo_pkg_dir, 'world', 'empty.world'),
		description='Full path to world model file to load')

	# Specify the actions
	start_gazebo_server_cmd = ExecuteProcess(
		cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world],
		cwd=[mecanum_gazebo_pkg_dir], output='screen')

	start_gazebo_client_cmd = ExecuteProcess(
		condition=IfCondition(PythonExpression(['not ', headless])),
		cmd=['gzclient'],
		cwd=[mecanum_gazebo_pkg_dir], output='screen')

	# Create the launch description and populate
	ld = LaunchDescription()

	# Declare the launch options
	ld.add_action(declare_use_sim_time_cmd)
	ld.add_action(declare_simulator_cmd)
	ld.add_action(declare_world_cmd)

	# Add any conditioned actions
	ld.add_action(start_gazebo_server_cmd)
	ld.add_action(start_gazebo_client_cmd)

	return ld