import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'metr4202_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lingxu',
    maintainer_email='lingxu@todo.todo',
    description='Navigation nodes for the METR4202 Team 13 project.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_manager = metr4202_navigation.mission_manager:main',
            'exploration = metr4202_navigation.exploration:main',
            'coverage = metr4202_navigation.coverage:main',
            'recovery = metr4202_navigation.recovery:main',
        ],
    },
)
