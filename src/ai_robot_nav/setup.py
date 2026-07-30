from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ai_robot_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    # Runtime deps are declared in package.xml so rosdep can resolve them.
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='LLM-based autonomous navigation with ROS 2 and Ollama',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ai_nav_node = ai_robot_nav.ai_nav_node:main',
            'safety_node = ai_robot_nav.safety_node:main'
        ],
    },
)
