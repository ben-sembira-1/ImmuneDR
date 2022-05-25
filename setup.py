from setuptools import setup, find_packages

setup(
    name="drones",
    version="0.0.1",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=[
        "pymavlink",
        "async_state_machine @ git+ssh://git@github.com/ori155/async_state_machine.git",
        "numpy>=1.20",  # minimal version with type support
    ],
    extras_require={"test": ["pytest", "pytest-repeat", "mypy"]},
)
