from setuptools import setup

package_name = 'camera_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='banana-killer',
    maintainer_email='sashagrachev@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_stand_up = camera_package.camera_stand_up:main',
            'camera_stand_up_combination = camera_package.camera_stand_up_combination:main',
        ],
    },
)
