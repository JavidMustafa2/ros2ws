from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
# This launch starts the nodes required for visualizing the sinSignal topic published. 
# The launch file does not start the servo subscriber.py node that should be done either seperately with ros2 run (to test with keyboard inputs)
# or by expanding this launch file.  

def generate_launch_description():

    config = os.path.join(
    get_package_share_directory('openrb_assignment'),
        'config',)
    
    return LaunchDescription([
        
        Node(
            package='openrb_assignment',
            executable='talker',
            name='sin_generator',
            parameters=[config + '/sin_generator_config.yaml'],
        ),
        Node(
            package='openrb_assignment',
            executable='listener',
            name='servo_subscriber',
            parameters=[config + '/servo_subscriber_config.yaml'],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_world_map",
            arguments=["0","0","0","0","0","0","world","map"],
        ),
        Node(
            package="openrb_assignment",
            executable="wave_tf_broadcaster",
            name="wave_tf_broadcaster",
            parameters=[{
                "parent_frame": "map",
                "child_frame": "sin_frame",
                "topic": "sinSignal",
                "rate_hz": 30.0,
            }],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
        ),
    ])