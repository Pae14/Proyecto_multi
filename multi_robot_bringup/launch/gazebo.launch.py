import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_path = get_package_share_directory('multi_robot_bringup')
    
    aws_models_paths = [
        os.path.join(pkg_path, 'ThirdParty/aws-robomaker-racetrack-world/models'),
        os.path.join(pkg_path, 'ThirdParty/aws-robomaker-small-warehouse-world/models'),
        os.path.join(pkg_path, 'ThirdParty/aws-robomaker-bookstore-world/models'),
        os.path.join(pkg_path, 'ThirdParty/aws-robomaker-small-house-world/models'),
        os.path.join(pkg_path, 'models')
    ]
    
    gz_resource_path = ':'.join(aws_models_paths)
    world = os.path.join(pkg_path, 'world', 'myworld.world')

    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([FindPackageShare('rover_description'), 'urdf', 'rover.xacro'])]),
        value_type=str
    )
    
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    
    spawn_rover = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'rover', '-z', '0.1'],
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=gz_resource_path),
        ExecuteProcess(cmd=['gz', 'sim', '-r', world], output='screen'),
        robot_state_pub,
        spawn_rover
    ])
