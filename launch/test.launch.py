from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='rokey', executable='go_main', name='go_main_node', output='screen', emulate_tty=True),
        Node(package='rokey', executable='box_seg', name='box_seg_node', output='log', emulate_tty=True),
        Node(package='rokey', executable='pill_seg', name='pill_seg_node', output='log', emulate_tty=True),
        
    ])

