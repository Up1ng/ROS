from glob import glob

from setuptools import find_packages, setup

package = 'lane_controller'
setup(
    name=package, version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package]),
        ('share/' + package, ['package.xml']),
        ('share/' + package + '/config', glob('config/*')),
        ('share/' + package + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], extras_require={'test': ['pytest']},
    maintainer='ROS team', maintainer_email='robotics-team@example.org',
    description='HSV lane following and time-aware PID for AutoRace.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['lane_controller = lane_controller.node:main']},
)
