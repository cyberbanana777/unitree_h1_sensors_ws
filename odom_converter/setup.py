from setuptools import setup

package_name = 'odom_converter'

setup(
    name=package_name,
    version='2.0.0',
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
    description='Converts Unitree H1 odometry (SportModeState + LowState) '
                'to nav_msgs/Odometry.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_converter_node = odom_converter.odom_converter_node:main',
        ],
    },
)
