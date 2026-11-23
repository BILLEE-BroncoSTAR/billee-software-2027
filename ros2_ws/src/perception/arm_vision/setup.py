import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'arm_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LucaLanzi',
    maintainer_email='luquilanzi@gmail.com',
    description='Arm RealSense D435i: driver launch, RGB-D input for CV nodes, ArUco detection, keyboard localization.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aruco_node = arm_vision.aruco_node:main',
            'aruco_generate = arm_vision.generate_markers:main',
            'keyboard_node = arm_vision.keyboard_node:main',
        ],
    },
)
