import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('multi_robot_bringup')
    world_name = 'bosque_ruinas_final'
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'gazebo.launch.py'))
    )

    # PUENTE MAESTRO: Corregido remapeo de TF y Topics
    bridge_maestro = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/uav_cerberus/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/uav/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/rover/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/rover/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/rover/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/uav/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/rover/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/uav_cerberus/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            f'/world/{world_name}/model/uav_cerberus/link/base_link/sensor/camera_front/image@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        remappings=[
            ('/model/uav_cerberus/odometry', '/uav/odom'),
            ('/model/rover/odometry', '/rover/odom'),
            ('/model/rover/cmd_vel', '/rover/cmd_vel'),
            ('/model/uav_cerberus/tf', '/tf'),
            (f'/world/{world_name}/model/uav_cerberus/link/base_link/sensor/camera_front/image', '/uav/camera/image_raw')
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    vision = Node(package='uav_vision', executable='detector_objetos', output='screen', parameters=[{'use_sim_time': True}])
    seguidor = Node(package='rover_navigation', executable='seguidor_dron.py', output='screen', parameters=[{'use_sim_time': True}])
    autonomo = Node(package='uav_vision', executable='dron_autonomo', output='screen', parameters=[{'use_sim_time': True}])

    # SLAM para el Rover
    slam_rover = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox_rover',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'base_frame': 'rover/base_link',
            'odom_frame': 'rover/odom',
            'map_frame': 'map',
            'scan_topic': '/rover/scan',
            'mode': 'mapping'
        }]
    )

    # SLAM para el Dron con remapeos de servicio completos
    slam_uav = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox_uav',
        output='screen',
        remappings=[
            ('/map', '/uav/map'),
            ('/map_metadata', '/uav/map_metadata'),
            ('/slam_toolbox/graph_visualization', '/uav/graph_visualization'),
            ('/slam_toolbox/get_map', '/uav/get_map'),
            ('/slam_toolbox/dynamic_map', '/uav/dynamic_map'),
        ],
        parameters=[{
            'use_sim_time': True,
            'base_frame': 'uav_cerberus/base_link',
            'odom_frame': 'world',
            'map_frame': 'uav_map',
            'scan_topic': '/uav/scan',
            'mode': 'mapping'
        }]
    )

    # Conector de mundos (Map -> UAV_Map -> World)
    # Eliminamos world_to_map porque causa un conflicto de doble padre para 'world'
    uav_map_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'uav_map'],
        parameters=[{'use_sim_time': True}]
    )

    vision = Node(package='uav_vision', executable='detector_objetos', output='screen')
    seguidor = Node(package='rover_navigation', executable='seguidor_dron.py', output='screen')
    #wander = Node(package='rover_navigation', executable='wander.py', parameters=[{'robot': 'rover'}], output='screen')
    autonomo = Node(package='uav_vision', executable='dron_autonomo', output='screen')

    return LaunchDescription([
        gazebo,
        bridge_maestro,
        slam_rover,
        slam_uav,
        uav_map_to_map,
        TimerAction(period=5.0, actions=[vision, seguidor, autonomo]),
    ])
