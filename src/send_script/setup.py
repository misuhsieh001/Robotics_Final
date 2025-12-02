from setuptools import find_packages, setup

package_name = 'send_script'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'send_script = send_script.send_script:main',
        'image_sub = send_script.image_sub:main',
        'cube_stacker_logic = send_script.cube_stacker_logic:main',
        'tm5_calibration_capture = send_script.tm5_calibration_capture:main',
        'calibration_process = send_script.calibration_process:main',
        ],
    },
)
