#!/usr/bin/env python3.8


from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['pick_n_place'],
    package_dir={'': 'src'},
)

setup(**setup_args)