from setuptools import setup
import os
from glob import glob

package_name = 'uav_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ¡Esta línea es clave para que ROS encuentre vision.launch.py!
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='paula',
    maintainer_email='paula@todo.todo',
    description='Paquete de visión artificial para el UAV',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector_objetos = uav_vision.detector_objetos:main',
            'dron_autonomo = uav_vision.dron_autonomo:main',
            'teleop_dron = uav_vision.teleop_dron:main'
        ],
    },
)