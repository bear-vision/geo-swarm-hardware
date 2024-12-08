from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'laptop_master'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(
        exclude=['test'],
        include=[
            package_name,
            f'{package_name}.*',
            f'{package_name}.behaviours',
            f'{package_name}.behaviours.*',
            f'{package_name}.decorators',
            f'{package_name}.decorators.*'
        ]
    ),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools', 'px4_msgs'],
    zip_safe=True,
    maintainer='ale',
    maintainer_email='alexandrazj28@gmail.com',
    description='Node with behaviour tree for drone autonomous logic',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laptop_master_node = laptop_master.laptop_master_node:main',
            'dummy_pub_node = laptop_master.dummy_pub_node:main',
            'dummy_action_server_node = laptop_master.dummy_action_server_node:main'
        ],
    },
)
