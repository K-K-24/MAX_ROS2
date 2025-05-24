from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'roomba_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
               (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='krish',
    maintainer_email='krishna24002@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_reader = roomba_bringup.sensor_reader:main',
            'imu_node = roomba_bringup.imu_node:main',
            'odometry_node = roomba_bringup.odometry_node:main',
            'motor_driver = roomba_bringup.motor_driver:main',
        'test_motors = roomba_bringup.test_motor_control:main',  
        'pid_test = roomba_bringup.pid_test_node:main', 
        'encoder_validator = roomba_bringup.encoder_val_node:main',
        ],
    },
)
