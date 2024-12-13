from setuptools import find_packages, setup

package_name = 'simple_rpi'

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
    maintainer='ale',
    maintainer_email='alexandrazj28@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_rpi_node = simple_rpi.simple_rpi_node:main',
            'dummy_pub_node = simple_rpi.dummy_pub_node:main',
            'dummy_servo_srv_node = simple_rpi.dummy_servo_srv_node:main'
        ],
    },
)
