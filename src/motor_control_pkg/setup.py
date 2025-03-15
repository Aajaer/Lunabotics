from setuptools import find_packages, setup

package_name = 'motor_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install package.xml
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        ('share/' + package_name + '/launch', ['launch/motor_control_launch.py']),
        ('share/' + package_name + '/launch', ['launch/slam_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aajaer',
    maintainer_email='aajaer@todo.todo',
    description='Motor control package for obstacle detection',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_motor_control = motor_control_pkg.lidar_motor_control:main',
        ],
    },
)
