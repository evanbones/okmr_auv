from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'okmr_controls'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='okmr',
    maintainer_email='okmr',
    description='package used by okanagan marine robotics for L1 software control systems',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid=okmr_controls.PIDController:main',
            'pid_combiner=okmr_controls.PIDCombiner:main',
        ],
    },
)
