from setuptools import setup

package_name = 'path_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jannik',
    maintainer_email='privat@jannikheinen.de',
    description='Path planning service node for ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planning_node = path_planning.path_planning_node:main',
            'plan_path_paint_test = path_planning.plan_path_paint_test:main'
        ],
    },
)
