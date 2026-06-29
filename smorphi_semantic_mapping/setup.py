from setuptools import find_packages, setup

package_name = 'smorphi_semantic_mapping'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/semantic_mapping.launch.py']),
        ('share/' + package_name + '/models', ['models/best.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='smorphi',
    maintainer_email='smorphi@todo.todo',
    description='YOLOv8 + LiDAR semantic mapping for Smorphi.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'semantic_mapper = smorphi_semantic_mapping.semantic_mapper_node:main',
        ],
    },
)
