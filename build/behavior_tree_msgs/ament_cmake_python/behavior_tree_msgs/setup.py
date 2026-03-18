from setuptools import find_packages
from setuptools import setup

setup(
    name='behavior_tree_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('behavior_tree_msgs', 'behavior_tree_msgs.*')),
)
