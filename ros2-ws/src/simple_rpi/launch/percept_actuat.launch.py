from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

package = 'simple_rpi'
def generate_launch_description():
    # Path to the RealSense launch file
    realsense_launch_file = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    return LaunchDescription([
        # Include the RealSense launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_file),
            launch_arguments={
                'color_width': '424',
                'color_height': '240',
                'depth_width': '424',
                'depth_height': '240',
                'align_depth': 'true',
                'depth_fps': '6',
                'color_fps': '6',
                'enable_rgbd': 'true',
                'enable_sync': 'true',
                'align_depth.enable': 'true',
                'enable_color': 'true',
                'enable_depth': 'true'
            }.items()
        ),
        
        # Add dirt_detector node
        Node(
            package=package,
            executable='dirt_detector',
            name='dirt_detector',
            output='screen'
        ),
        
        # Add tower_detector node
        Node(
            package=package,
            executable='tower_detector',
            name='tower_detector',
            output='screen'
        ),
        
        # Add perception_stuff node
        Node(
            package=package,
            executable='perception_stuff',
            name='perception_stuff',
            output='screen'
        ),

        # Add Servo node
        Node(
            package=package,
            executable='servo_node',
            name='servo_node',
            output='screen'
        ),
    ])
