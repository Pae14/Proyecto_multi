from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterValue
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('abb_irb120_description')
    xacro_file_path = os.path.join(pkg_share, 'urdf', 'irb120_3_58_macro.xacro')

    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file_path]), value_type=str
    )

    #El state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': True}],
        output='screen'
    )

    #Lanzar con gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    #Generar al robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'abb_irb120'],
        output='screen'
    )

    #Lanzar controladores
    load_joint_state = TimerAction(
        period=5.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        )]
    )

    load_arm_controller = TimerAction(
        period=7.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_controller"],
        )]
    )

    #El robot studio
    bridge_node = TimerAction(
        period=10.0,
        actions=[Node(
            package='prueba',
            executable='prueba_comunicacion',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )]
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot,
        load_joint_state,
        load_arm_controller,
        bridge_node
    ])