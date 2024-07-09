from setuptools import setup, find_packages

setup(
    name="stabilizer",
    packages=find_packages(),
    # Keep versions in Cargo.toml and py/setup.py synchronized.
    version="0.9.0",
    description="Stabilizer Utilities",
    author="QUARTIQ GmbH",
    license="MIT",
    python_requires=">=3.8",
    install_requires=[
        "numpy",
        "scipy",
        "matplotlib",
        "gmqtt",
        # Keep this synced with the miniconf version in Cargo.toml
        "miniconf-mqtt@git+https://github.com/quartiq/miniconf@mqtt-rs-rework#subdirectory=py/miniconf-mqtt",
    ],
)
