from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

package_name = 'monodepth_ros'  # this should be the name of your package

setup_args = generate_distutils_setup(
    packages=[package_name],
    package_dir={'': 'src'},  # usually 'src' or '.'
    requires=['std_msgs', 'rospy']  # add here all the ros dependencies
)

setup(**setup_args)

