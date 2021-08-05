#!/usr/bin/python3
"""
Author: Vertigo Designs, Ryan Summers

Description: Setup file for Miniconf packaging.
"""
from setuptools import setup, find_packages

# Load the version string from the version file.
execfile('miniconf/version.py')

setup(name='miniconf',
      version=__version__,
      author='Ryan Summers, Robert Jördens',
      description='Utilities for configuring Miniconf-configurable devices',
      url='https://github.com/quartiq/miniconf',
      packages=find_packages(),
      install_requires=[
        'gmqtt'
      ],
)
