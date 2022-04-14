from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# Based on https://roboticsbackend.com/ros-import-python-module-from-another-package/



d = generate_distutils_setup(
    packages=['franka_modules'],
    package_dir={'': 'src'}
)
setup(**d)