from setuptools import setup
import os
from glob import glob

package_name = 'proyecto'

def package_files(directory):
    paths = []
    for root, _, files in os.walk(directory):
        for f in files:
            full_path = os.path.join(root, f)
            install_path = os.path.join(
                'share', package_name, os.path.relpath(root, package_name)
            )
            paths.append((install_path, [full_path]))
    return paths


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],

    data_files=[
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),

        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*.world')),

        *package_files('models'),  # 👈 aquí se usa

        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ruth',
    maintainer_email='ruth@todo.todo',
    description='TODO',
    license='TODO',
    entry_points={
        'console_scripts': [],
    },
)