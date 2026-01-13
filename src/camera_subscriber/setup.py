from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'camera_subscriber'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # REQUIRED by ROS2
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name,
            ['package.xml']),

        # launch files  👈 TO JEST KLUCZOWE
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='pw.lubon@gmail.com',
    description='Camera subscriber with ArUco control',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'camera_node = camera_subscriber.camera_node:main',
        ],
    },
)

