import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('ai_robot_nav'), 'config', 'ai_nav_params.yaml')

    params_file = LaunchConfiguration('params_file')
    ollama_url = LaunchConfiguration('ollama_url')
    ollama_model = LaunchConfiguration('ollama_model')
    use_image = LaunchConfiguration('use_image')
    use_sim_time = LaunchConfiguration('use_sim_time')
    cmd_stamped = LaunchConfiguration('cmd_stamped')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the parameter YAML for both nodes.'),
        DeclareLaunchArgument(
            'ollama_url',
            default_value='http://localhost:11434/api/generate',
            description="Ollama generate endpoint; use the host's LAN address off-board."),
        DeclareLaunchArgument(
            'ollama_model',
            default_value='llava:7b',
            description='Ollama model tag to query.'),
        DeclareLaunchArgument(
            'use_image',
            default_value='true',
            description='Send camera frames to the model. False skips the model entirely.'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Follow /clock. Set true with Gazebo so the staleness checks '
                        'and watchdogs track simulation time; a sim running below '
                        'real time otherwise looks like dead sensors.'),
        DeclareLaunchArgument(
            'cmd_stamped',
            default_value='true',
            description='Publish TwistStamped on /cmd_vel (Gazebo Sim 8 / Jazzy). '
                        'Set false for base drivers that subscribe to Twist.'),

        Node(
            package='ai_robot_nav',
            executable='ai_nav_node',
            name='ai_nav_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'ollama_url': ollama_url,
                    'ollama_model': ollama_model,
                    'use_image': ParameterValue(use_image, value_type=bool),
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                },
            ],
        ),
        Node(
            package='ai_robot_nav',
            executable='safety_node',
            name='safety_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                    'cmd_stamped': ParameterValue(cmd_stamped, value_type=bool),
                },
            ],
        ),
    ])
