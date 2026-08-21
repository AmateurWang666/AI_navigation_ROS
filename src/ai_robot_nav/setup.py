from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['ai_robot_nav'],
    package_dir={'': '.'},
)

setup(**setup_args)
