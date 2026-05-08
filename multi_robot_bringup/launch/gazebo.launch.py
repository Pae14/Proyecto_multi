from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

import os
import yaml


def generate_launch_description():

    pkg_path = get_package_share_directory('multi_robot_bringup')

    # =========================
    # GAZEBO RESOURCE PATH
    # =========================

    aws_models_paths = [
        os.path.join(pkg_path, 'ThirdParty/aws-robomaker-racetrack-world/models'),
        os.path.join(pkg_path, 'ThirdParty/aws-robomaker-small-warehouse-world/models'),
        os.path.join(pkg_path, 'ThirdParty/aws-robomaker-bookstore-world/models'),
        os.path.join(pkg_path, 'ThirdParty/aws-robomaker-small-house-world/models'),
        os.path.join(pkg_path, 'models')
    ]

    gz_resource_path = ':'.join(aws_models_paths)

    world_path = os.path.join(pkg_path, 'world', 'myworld.world')

    # =========================
    # ROBOT CONFIG
    # =========================

    robot_name = 'rover'

    x_pos = 0.0
    y_pos = 0.0

    # =========================
    # BRIDGE CONFIG YAML
    # =========================

    bridge_config = [
        {
            'ros_topic_name': '/clock',
            'gz_topic_name': '/clock',
            'ros_type_name': 'rosgraph_msgs/msg/Clock',
            'gz_type_name': 'gz.msgs.Clock',
            'direction': 'GZ_TO_ROS'
        },

        {
            'ros_topic_name': f'/{robot_name}/scan',
            'gz_topic_name': f'/{robot_name}/scan',
            'ros_type_name': 'sensor_msgs/msg/LaserScan',
            'gz_type_name': 'gz.msgs.LaserScan',
            'direction': 'GZ_TO_ROS'
        },

        {
            'ros_topic_name': f'/{robot_name}/cmd_vel',
            'gz_topic_name': f'/{robot_name}/cmd_vel',
            'ros_type_name': 'geometry_msgs/msg/Twist',
            'gz_type_name': 'gz.msgs.Twist',
            'direction': 'ROS_TO_GZ'
        },

        {
            'ros_topic_name': f'/{robot_name}/odom',
            'gz_topic_name': f'/{robot_name}/odom',
            'ros_type_name': 'nav_msgs/msg/Odometry',
            'gz_type_name': 'gz.msgs.Odometry',
            'direction': 'GZ_TO_ROS'
        },

        {
            'ros_topic_name': '/tf',
            'gz_topic_name': f'/{robot_name}/tf',
            'ros_type_name': 'tf2_msgs/msg/TFMessage',
            'gz_type_name': 'gz.msgs.Pose_V',
            'direction': 'GZ_TO_ROS'
        },
        {
            'ros_topic_name': '/tf_static',
            'gz_topic_name': f'/{robot_name}/tf_static', 
            'ros_type_name': 'tf2_msgs/msg/TFMessage',
            'gz_type_name': 'gz.msgs.Pose_V', 
            'direction': 'GZ_TO_ROS'
        },

        {
            'ros_topic_name': f'/{robot_name}/joint_states',
            'gz_topic_name': f'/world/bosque_ruinas_final/model/{robot_name}/joint_state',
            'ros_type_name': 'sensor_msgs/msg/JointState',
            'gz_type_name': 'gz.msgs.Model',
            'direction': 'GZ_TO_ROS'
        }
    ]

    # Guardar YAML temporal
    bridge_yaml = os.path.join('/tmp', f'{robot_name}_bridge.yaml')

    with open(bridge_yaml, 'w') as f:
        yaml.dump(bridge_config, f)

    # =========================
    # NODES
    # =========================

    nodes_list = []

    # Gazebo
    nodes_list.append(
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=gz_resource_path
        )
    )
    #este es para que encuentre la libreria del ros2 control
    nodes_list.append(
        SetEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value='/opt/ros/jazzy/lib' 
        )
    )

    nodes_list.append(
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_path],
            output='screen'
        )
    )
    robot_description = ParameterValue( #cargar el rover
        Command([
            'xacro ',
            PathJoinSubstitution([
                FindPackageShare('multi_robot_bringup'),
                'urdf',
                'rover_abb.xacro'
            ]), ' ',
            'prefix:=', f'{robot_name}/', ' ',
        ]),
        value_type=str
    )
    # Robot group
    robot_group = GroupAction([

        PushRosNamespace(robot_name),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', robot_name, '-x', '0', '-y', str(y_pos)]
        ),
    ])
    nodes_list.append(robot_group)
    nodes_list.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', str(y_pos), '0', '0', '0', '0', 'map', f'{robot_name}/odom']
        )
    )
    # Bridge
    nodes_list.append(
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters= [{'config_file': bridge_yaml, 'use_sim_time': True}]
        )
    )
    nodes_list.append(
        Node(
            package='rviz2',
            executable='rviz2',
            parameters=[{'use_sim_time': False}] #false para que no de problemas el rviz
        )
    )
    #cargar el controlador joint_state_broadcaster
    nodes_list.append(
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster', '-c', '/rover/controller_manager'],
            output='screen'
        )
    )
    #cargar el arm controlles
    nodes_list.append(
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller', '-c', '/rover/controller_manager'],
            output='screen'
        )
    )

    return LaunchDescription(nodes_list)