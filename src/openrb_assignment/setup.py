from setuptools import find_packages, setup

package_name = 'openrb_assignment'

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
    maintainer='javid',
    maintainer_email='jmustafa2011@hotmail.com',
    description='ROS2 package for ROS2 Tutorial Assignment for RWR2025',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': ['talker = openrb_assignment.sin_generator:main',
                            'listener = openrb_assignment.servo_subscriber:main',
        ],
    },
)
