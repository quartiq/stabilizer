from setuptools import setup, find_packages

import miniconf

setup(name='miniconf',
      version=miniconf.__version__,
      author='Ryan Summers, Robert Jördens',
      description='Utilities for configuring Miniconf-configurable devices',
      url='https://github.com/quartiq/miniconf',
      packages=find_packages(),
      install_requires=[
        'gmqtt'
      ],
)
