import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'zed_usb'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anton C.',
    maintainer_email='anton.2.clark@gmail.com',
    description='ZED 2i over plain USB with CPU stereo depth, for machines without an NVIDIA GPU.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'zed_usb_node = zed_usb.node:main',
        ],
    },
)
