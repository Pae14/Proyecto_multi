import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Mensaje de confirmación en consola
    print("\n" + "!"*60)
    print(">>> INICIANDO VERSION LIMPIA - SOLO PAQUETE 'PRUEBA' <<<")
    print("!"*60 + "\n")

    pkg_abb_description = get_package_share_directory('abb_irb120_description')
    
    # Procesado de URDF
    xacro_file = os.path.join(pkg_abb_description, 'urdf', 'abb.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str
    )

    # Nodo de Estado
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # Simulación
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'abb_irb120', '-z', '0.1'],
        output='screen'
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}]
    )

    # Controladores
    load_joint_state = TimerAction(
        period=5.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        )]
    )

    load_arm_controller = TimerAction(
        period=8.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_controller"],
        )]
    )


    return LaunchDescription([
        clock_bridge,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        load_joint_state,
        load_arm_controller,
        ])