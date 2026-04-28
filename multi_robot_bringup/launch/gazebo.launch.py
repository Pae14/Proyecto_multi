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
    world_name = 'bosque_ruinas_mezcla'

    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                FindPackageShare('rover_description'),
                'urdf',
                'rover.xacro'
            ])
        ]),
        value_type=str
    )
    robot=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    gazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'rover',
            '-x', '0',
            '-y', '0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # --- BRIDGES (PUENTES) PARA LOS ROBOTS PROFESIONALES ---
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            # ROVER (corregido)
            '/model/rover/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/rover/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',

            # DRON (déjalo si existe)
            #'/model/uav_cerberus/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',

            f'/world/{world_name}/model/uav_cerberus/link/base_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        remappings=[
            ('/model/rover/cmd_vel', '/rover/cmd_vel'),
            ('/model/rover/odometry', '/rover/odom'),

            ('/model/uav_cerberus/cmd_vel', '/uav/cmd_vel'),
            (f'/world/{world_name}/model/uav_cerberus/link/base_link/sensor/camera/image', '/uav/camera/image_raw'),
        ],
        output='screen'
    )


    return LaunchDescription([
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=gz_resource_path),
        
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world],
            output='screen'
        ),
        robot,
        gazebo,
        bridge
    ])
