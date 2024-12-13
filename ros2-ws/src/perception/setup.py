from setuptools import find_packages, setup

package_name = 'perception'

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
    maintainer='citris',
    maintainer_email='corygeodrone@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_perception = perception.drone_perception:main',
            'dirt_detector = perception.dirt_detector:main',
            'tower_detector = perception.tower_detector:main',
            'perception_stuff = perception.perception_stuff:main',
        ],
    },
)
