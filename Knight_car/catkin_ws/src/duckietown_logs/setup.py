from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['duckietown_logs'],
    package_dir={'': 'include'},
    install_requires = [
     'SystemCmd==1.2', 
     'ros_node_utils==1.0', 
     'ConfTools==1.8',
     'QuickApp==1.2.2',
     'Procgraph==1.10',
    ]
)

#print(setup_args)

#raise ValueError('are you even executing this?')
setup(**setup_args)
