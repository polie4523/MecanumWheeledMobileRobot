from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    
    #init_cond_launch_arg = DeclareLaunchArgument(
    #    'init_cond', default_value=TextSubstitution(text='[0.5, 0, 0]')
    #)
    #pid_gain_x_launch_arg = DeclareLaunchArgument(
    #    'pid_gain_x', default_value=TextSubstitution(text='[1, 0.25, 0.25]')
    #)
    #pid_gain_y_launch_arg = DeclareLaunchArgument(
    #    'pid_gain_y', default_value=TextSubstitution(text='[1, 0.25, 0.25]')
    #)
    #pid_gain_theta_launch_arg = DeclareLaunchArgument(
    #    'pid_gain_theta', default_value=TextSubstitution(text='[1, 0.25, 0.25]')
    #)
    
    return LaunchDescription([
        #init_cond_launch_arg,
        #pid_gain_x_launch_arg,
        #pid_gain_y_launch_arg,
        #pid_gain_theta_launch_arg,
        Node(
            package='MecanumWMR_control',
            executable='control_node',
            name='control_node',
            parameters=[{
                'init_cond': [0.0,0.0,0.0],
                'pid_gain_translation': [1.0,0.25,0.25],
                'pid_gain_rotation': [1.0,0.25,0.25],
            }]
        ),
    ])
