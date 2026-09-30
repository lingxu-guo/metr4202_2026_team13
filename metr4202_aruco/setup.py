import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'metr4202_aruco'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lingxu',
    maintainer_email='lingxu@todo.todo',
    description='ArUco detection and localization nodes for METR4202 Team 13.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aruco_detection = metr4202_aruco.aruco_detection:main',
            'aruco_localization = metr4202_aruco.aruco_localization:main',
        ],
    },
)
