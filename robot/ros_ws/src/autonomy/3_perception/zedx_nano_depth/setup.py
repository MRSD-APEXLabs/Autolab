from glob import glob

from setuptools import find_packages, setup

package_name = 'zedx_nano_depth'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name + '.raft_stereo': ['LICENSE']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.xml')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Arker123',
    maintainer_email='arnavkha@andrew.cmu.edu',
    description='Depth image from the ZED X Nano stereo feed (RAFT-Stereo or SGBM)',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'zedx_nano_depth_node = zedx_nano_depth.depth_node:main',
        ],
    },
)
