from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'people'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'foxglove'), glob('foxglove/*')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LucaLanzi',
    maintainer_email='luquilanzi@gmail.com',
    description='OpenPose body keypoints lifted to 3D skeletons in odom.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'openpose_node = people.openpose_node:main',
        ],
    },
)
