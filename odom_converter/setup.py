from setuptools import setup

package_name = 'odom_converter'

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
    description='The package includes node what converts odometry from a custom' \
    ' msg by Unitree to a standard odometry msg',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_convereter_node = odom_converter.odom_convereter_node:main'
        ],
    },
)
