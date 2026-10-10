from glob import glob

from setuptools import find_packages, setup

package_name = 'image_stream_server'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.xml')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='juan',
    maintainer_email='jpuerto96@hotmail.com',
    description='Lazy MJPEG-over-HTTP server for ROS image topics',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'image_stream_server_node = image_stream_server.server_node:main',
        ],
    },
)
