from setuptools import find_packages, setup
import os

package_name = 'servo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Explicitly include the single service file
        (os.path.join('share', package_name, 'srv'), ['srv/ServoControl.srv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alexandra',
    maintainer_email='alexandrazj28@gmail.com',
    description='Package that controls a servo by sending corresponding PWM signal',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_node = servo.servo_node:main'
        ],
    },
)
