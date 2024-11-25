from setuptools import find_packages, setup

package_name = 'laptop_master'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'laptop_master_node = laptop_master.laptop_master_node:main'
        ],
    },
)
