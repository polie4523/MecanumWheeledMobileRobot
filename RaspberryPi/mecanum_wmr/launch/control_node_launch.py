from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='mecanum_wmr',
            executable='control_node',
            name='control_node',
            parameters=[{
                'init_cond': [0.0,0.0,0.0],
                'pid_gain_translation': [1.0,0.25,0.25],
                'pid_gain_rotation': [1.0,0.25,0.25],
            }]
        ),
    ])
