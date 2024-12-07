from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_planning',
            executable='path_planning_node',
            name='path_planning_node'
        ),
        Node(
            package='laptop_master',
            executable='dummy_pub_node',
            name='dummy_pub_node'
        ),
        # Node(
        #     package='laptop_master',
        #     executable='dummy_action_server_node',
        #     name='dummy_action_server_node'
        # )
        Node(
            package='rpi_master',
            executable='rpi_master_node',
            name='rpi_master_node'
        )
    ])
