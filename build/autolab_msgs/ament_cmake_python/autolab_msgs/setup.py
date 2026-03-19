from setuptools import find_packages
from setuptools import setup

setup(
    name='autolab_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('autolab_msgs', 'autolab_msgs.*')),
)
