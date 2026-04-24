from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('proyecto')

    world = os.path.join(pkg_path, 'worlds', 'myworld.world')
    models = os.path.join(pkg_path, 'models')

    gz_path = models

    
    return LaunchDescription([

        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=gz_path
        ),

        ExecuteProcess(
            cmd=['gz', 'sim', world],
            output='screen'
        )
    ])