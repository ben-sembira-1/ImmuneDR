from setuptools import setup, find_packages

setup(
    name='drones',
    version='0.0.1',
    packages=find_packages(where="src"),
    package_dir={
        "": "src"
    },
    install_requires=[
        "pymavlink"
    ],
    extras_require= {
        "test": "pytest"
    }
)