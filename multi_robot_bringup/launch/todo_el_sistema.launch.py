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

    # Bridge Maestro: Forzamos el remapeo de la cámara del dron
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
            # Cámara del dron con remapeo explícito
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

    return LaunchDescription([
        gazebo,
        bridge_maestro,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'uav_map'],
            parameters=[{'use_sim_time': True}]
        ),
        TimerAction(period=5.0, actions=[vision, seguidor, autonomo]),
    ])
