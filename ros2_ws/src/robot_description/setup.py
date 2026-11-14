from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_description'


def files(pattern):
    # Real files only: `colcon build --symlink-install` leaves links to deleted source
    # files in build/<pkg>/, and installing a dangling link fails the build.
    return [f for f in glob(pattern) if os.path.isfile(f)]


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'urdf'), files('urdf/*')),
        (os.path.join('share', package_name, 'config'), files('config/*')),
        (os.path.join('share', package_name, 'meshes'), files('meshes/*')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros_user',
    maintainer_email='Vernonicuhhh@protonmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
