import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'px4_lidar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 'px4_lidar', 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bravo1311',
    maintainer_email='agrawalkartik1999@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'px4_odom_tf = px4_lidar.px4_odom_tf:main',
            'scan_frame_fix = px4_lidar.scan_frame_fix:main',
            'safety_vel_filter = px4_lidar.safety_vel_filter:main',
            'gz_odom_tf = px4_lidar.gz_odom_tf:main',
        ],
    },
)
