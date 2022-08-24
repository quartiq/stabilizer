from setuptools import setup, find_packages

setup(name="stabilizer",
      packages=find_packages(),
      # Keep versions in Cargo.toml and py/setup.py synchronized.
      version="0.7.0",
      description="Stabilizer Utilities",
      author="QUARTIQ GmbH",
      license="MIT",
      install_requires=[
            "numpy",
            "scipy",
            "matplotlib",
            "gmqtt",
            "miniconf-mqtt@git+https://github.com/quartiq/miniconf@develop#subdirectory=py/miniconf-mqtt"
      ])
