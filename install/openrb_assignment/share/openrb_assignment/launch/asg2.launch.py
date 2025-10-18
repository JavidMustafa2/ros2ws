from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('openrb_assignment'),
        'config',
   
    )
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
        )
    ])

