from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'camera_stream'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros_user',
    maintainer_email='adpoitras@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'csi_compressed_video_pub = camera_stream.csi_compressed_video_publisher:main',
            'yolov8_detector = camera_stream.yolov8_object_detector:main',
        ],
    },
)
