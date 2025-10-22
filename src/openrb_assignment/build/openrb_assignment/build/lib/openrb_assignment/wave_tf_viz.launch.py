from launch import LaunchDescription
from launch_ros.actions import Node

# This launch starts the nodes required for visualizing the sinSignal topic published. 
# The launch file does not start the servo subscriber.py node that should be done either seperately with ros2 run (to test with keyboard inputs)
# or by expanding this launch file.  

def generate_launch_description():
    return LaunchDescription([
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