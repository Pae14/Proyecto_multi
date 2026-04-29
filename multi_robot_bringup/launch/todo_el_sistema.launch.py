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

    # PUENTE SIMPLIFICADO: Probamos rutas cortas que suelen ser más estables
    bridge_maestro = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/uav_cerberus/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/rover/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/rover/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            f'/world/{world_name}/model/uav_cerberus/link/base_link/sensor/camera_front/image@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        remappings=[
            ('/model/uav_cerberus/odometry', '/uav/odom'),
            ('/model/rover/odometry', '/rover/odom'),
            ('/model/rover/cmd_vel', '/rover/cmd_vel'),
            (f'/world/{world_name}/model/uav_cerberus/link/base_link/sensor/camera_front/image', '/uav/camera/image_raw')
        ],
        output='screen'
    )

    vision = Node(package='uav_vision', executable='detector_objetos', output='screen')
    seguidor = Node(package='rover_navigation', executable='seguidor_dron.py', output='screen')

    return LaunchDescription([
        SetEnvironmentVariable(name='PYTHONPATH', value=os.environ.get('PYTHONPATH', '') + ':/home/paula/venv/lib/python3.12/site-packages'),
        gazebo,
        bridge_maestro,
        TimerAction(period=4.0, actions=[vision, seguidor])
    ])
