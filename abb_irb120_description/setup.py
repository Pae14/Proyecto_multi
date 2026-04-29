from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'abb_irb120_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), #Archivos launch
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'meshes/visual'), glob('meshes/visual/*.stl')),
        (os.path.join('share', package_name, 'meshes/collision'), glob('meshes/collision/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lucia',
    maintainer_email='lucia@todo.todo',
    description='Paquete de descripcion del ABB IRB 120 para el proyecto de logistica',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)