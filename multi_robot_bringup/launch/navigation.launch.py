from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        namespace='rover',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    nav2 = Node(
        package='nav2_bringup',
        executable='bringup_launch.py',
        namespace='rover',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    nav2_delayed = TimerAction(
        period=5.0,
        actions=[nav2]
    )

    return LaunchDescription([
        slam,
        nav2_delayed
    ])