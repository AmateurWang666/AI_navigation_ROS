"""ament_python 包的构建配置。

launch 与 config 目录必须显式装进 share/：ai_nav.launch.py 是通过
get_package_share_directory 定位默认参数文件的，漏装会让节点起不来。
"""

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ai_robot_nav'

setup(
    name=package_name,
    version='0.0.1',
    # 排除 test：单元测试由 colcon test 直接从源码目录跑，不需要安装。
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament 索引标记，ros2 命令行据此发现本包。
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 用 glob 而不是逐个列出：新增 launch 或参数文件时不必再改这里。
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    # 运行时依赖统一声明在 package.xml，交给 rosdep 解析；这里只留 setuptools，
    # 避免同一份依赖在两处各写一遍还写得不一致。
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='LLM-based autonomous navigation with ROS 2 and Ollama',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        # 这两个名字就是 ros2 run / launch 里用的 executable 名。
        'console_scripts': [
            'ai_nav_node = ai_robot_nav.ai_nav_node:main',
            'safety_node = ai_robot_nav.safety_node:main'
        ],
    },
)
