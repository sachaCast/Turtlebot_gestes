from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='turtlebot3_autonomous',
            executable='obstacle_avoider',
            name='movement_logic'
        ),

        Node(
            package='ia_turtlebot_vision',
            executable='gesture_detector_node',
            name='gesture_vision'
        ),

        Node(
            package='robot_supervisor',
            executable='supervisor',
            name='brain'
        ),

    ])
