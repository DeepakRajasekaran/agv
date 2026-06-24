import os
from glob import glob
from setuptools import setup

package_name = 'diff_drive_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Differential drive controller node: cmd_vel/drive_rpm in, cmd_rpm/odom out.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diff_drive_controller_node = diff_drive_control.diff_drive_controller_node:main',
            'fake_drive_rpm_node = diff_drive_control.fake_drive_rpm_node:main',
        ],
    },
)
