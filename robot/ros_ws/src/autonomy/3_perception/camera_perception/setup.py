from glob import glob

from setuptools import find_packages, setup

package_name = 'camera_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'requirements.txt']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.xml')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Arker123',
    maintainer_email='arnavkha@andrew.cmu.edu',
    description='AprilTags, YOLO detections and a point cloud from the ZED X / ZED X Nano stereo depth',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'perception_node = camera_perception.perception_node:main',
        ],
    },
)
