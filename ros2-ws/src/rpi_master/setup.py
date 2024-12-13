from setuptools import find_packages, setup

package_name = 'rpi_master'

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
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rpi_master_node = rpi_master.rpi_master_node:main',
            'rpi_test_node = rpi_master.rpi_test_node:main'
        ],
    },
)
