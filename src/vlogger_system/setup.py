from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vlogger_system'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='vincent19980702@gmail.com',
    description='TM5-900 Indoor Vlogger System - Automated vlogging with face tracking and gesture control',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vlogger_control = vlogger_system.vlogger_control:main',
            'test_camera = vlogger_system.test_camera:main',
            'test_movement = vlogger_system.test_movement:main',
        ],
    },
)
