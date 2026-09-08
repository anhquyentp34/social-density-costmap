#!/usr/bin/env python3
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['social_metrics_calculator'],
    package_dir={'': 'src'},
    scripts=[
        'scripts/sgi_calculator.py',
        'scripts/sii_calculator.py'
    ]
)

setup(**setup_args) 