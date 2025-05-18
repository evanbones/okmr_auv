from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'okmr_hardware_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*.yaml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='erykhalicki0@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "serial_output=okmr_hardware_interface.serial_output_basic:main",
            "dvl_driver=okmr_hardware_interface.dvl_driver:main",
            "dvl_dummy_driver=okmr_hardware_interface.dvl_dummy:main",
            "temp_sensor=okmr_hardware_interface.temp_sensor:main",
        ],
    },
)
