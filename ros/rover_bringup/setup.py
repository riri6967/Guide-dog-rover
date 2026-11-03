from glob import glob

from setuptools import setup

package_name = 'rover_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/udev', glob('udev/*.rules')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RiRi',
    maintainer_email='2006rishikumar@gmail.com',
    description='Starts the Guide Dog Rover on the Raspberry Pi: odometry, IMU, EKF, LiDAR, camera',
    license='MIT',
    entry_points={
        'console_scripts': [
            'wheel_odometry = rover_bringup.wheel_odometry:main',
            'mpu6050_imu = rover_bringup.mpu6050_imu:main',
        ],
    },
)
