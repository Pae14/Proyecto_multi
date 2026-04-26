from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

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

    return LaunchDescription([

        # Publica estados de joints (ruedas)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace='rover'
        ),

        # Publica TF desde URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='rover',
            parameters=[{
                'robot_description': robot_description
            }]
        )
    ])