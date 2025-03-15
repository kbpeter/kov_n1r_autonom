from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='szenzor_rendszer',
            executable='szenzor_node',
            output='screen'
        ),
        Node(
            package='szenzor_rendszer',
            executable='figyelo_node',
            output='screen'
        ),
        Node(
            package='szenzor_rendszer',
            executable='adat_megjelenito',
            output='screen'
        )
    ])
