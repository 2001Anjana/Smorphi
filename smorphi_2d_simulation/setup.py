import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'smorphi_2d_simulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'map'), glob('map/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anjana',
    maintainer_email='2001wickramaarachchi@gmail.com',
    description='2D simulation for Smorphi auto shape transformation (O<->I)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_robot_node = smorphi_2d_simulation.sim_robot_node:main',
            'sim_lidar_node = smorphi_2d_simulation.sim_lidar_node:main',
            'shape_transform_manager = smorphi_2d_simulation.shape_transform_manager:main',
        ],
    },
)
