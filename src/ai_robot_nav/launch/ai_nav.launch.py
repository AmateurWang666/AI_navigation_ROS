"""同时启动 ai_nav_node 与 safety_node 的 launch 文件。

两个节点必须成对启动：ai_nav_node 只发建议速度到 /ai_cmd_vel，真正驱动底盘的
/cmd_vel 由 safety_node 独占发布。单独起 ai_nav_node 机器人不会动，单独起
safety_node 则会因为收不到建议速度而一直停车。

参数的来源有两层：先加载 params_file（默认是本包安装出来的 YAML），再用这里
声明的 launch 参数覆盖其中几个最常需要临时改的项。因此改一次性设置用命令行，
改长期配置改 YAML。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 从 share 目录取参数文件，而不是拼源码路径：只有安装后的副本才是运行时真正
    # 生效的那一份。
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
                # 列表顺序即优先级：后面的字典覆盖前面 YAML 里的同名项。
                params_file,
                {
                    'ollama_url': ollama_url,
                    'ollama_model': ollama_model,
                    # launch 参数从命令行来时一律是字符串，必须显式声明类型，
                    # 否则布尔参数会以 "true" 这种字符串形式传下去而报类型错误。
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
                    # use_sim_time 必须两个节点保持一致：一个用仿真时钟、另一个用
                    # 墙上时钟，超时判断就会互相矛盾。
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                    'cmd_stamped': ParameterValue(cmd_stamped, value_type=bool),
                },
            ],
        ),
    ])
