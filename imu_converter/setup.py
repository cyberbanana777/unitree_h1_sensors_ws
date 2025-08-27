from setuptools import setup

package_name = 'imu_converter'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='banana-killer',
    maintainer_email='sashagrachev2005@gmail.com',
    description='The package for convertation IMU data from custom msg from Unitree to standart ros2 imu-msgs (sensors_msgs/msg/IMU) ',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_converter_node = imu_converter.imu_converter_node:main'
        ],
    },
)
