from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lidar_gap_measure'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anjana',
    maintainer_email='2001wickramaarachchi@gmail.com',
    description='Measures narrow gap width using RPLidar /scan and publishes marker',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gap_width_node = lidar_gap_measure.gap_width_node:main',
        ],
    },
)

