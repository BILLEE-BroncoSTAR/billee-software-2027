import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'arm_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros_user',
    maintainer_email='adpoitras@gmail.com',
    description='BILLEE arm URDF (placeholder model until the new arm is imported), mounted on '
                'the rover with the RealSense D435i on its end effector.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'import_urdf = arm_description.import_urdf:main',
            'joint_state_restamp = arm_description.joint_state_restamp:main',
        ],
    },
)
