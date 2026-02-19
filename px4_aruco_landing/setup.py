from setuptools import find_packages, setup

package_name = 'px4_aruco_landing'

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
        'aruco_detector = px4_aruco_landing.aruco_detector:main',
        'autoland_twist_publisher = px4_aruco_landing.autoland_twist_publisher:main',
    ],
},

)
