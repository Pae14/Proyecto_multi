from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        namespace='rover',
        parameters=[{
            'use_sim_time': True
        }],
        output='screen'
    )

    return LaunchDescription([
        slam
    ])