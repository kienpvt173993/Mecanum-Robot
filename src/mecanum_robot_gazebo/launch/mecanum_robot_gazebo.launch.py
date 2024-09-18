import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='use_sim_time')

    robot_description_pkg_dir = get_package_share_directory('mecanum_robot_description')
    robot_gazebo_pkg_dir = get_package_share_directory('mecanum_robot_gazebo')
    
    launch_des_file_name = 'mecanum_robot_description.launch.py'
    launch_gazebo_file_name = 'empty_world.launch.py'
    current_gazebo_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    path_to_models = os.path.join(robot_description_pkg_dir, 'models')
    if current_gazebo_path == '':
        os.environ['GAZEBO_MODEL_PATH'] = path_to_models
    elif path_to_models not in current_gazebo_path:
        os.environ['GAZEBO_MODEL_PATH'] = f'{path_to_models}:{current_gazebo_path}'
    print(f"Path to stl models: {os.environ['GAZEBO_MODEL_PATH']}")
    return LaunchDescription([
        declare_use_sim_time_cmd,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'initial_x',
            default_value='0.0',
            description='Start pose x of robot in gazebo'),
        DeclareLaunchArgument(
            'initial_y',
            default_value='0.0',
            description='Start pose y of robot in gazebo'),
        DeclareLaunchArgument(
            'initial_yaw',
            default_value='0.0',
            description='Start pose yaw of robot in gazebo'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(robot_description_pkg_dir, 'launch', launch_des_file_name)),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(robot_gazebo_pkg_dir, 'launch', launch_gazebo_file_name)),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        Node(package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-topic', 'robot_description',
                        '-entity', 'zm_robot',
                        '-x',      initial_x,
                        '-y',      initial_y,
                        '-Y',      initial_yaw,
                        '-z',      '0.001'],
            output='screen')
    ])