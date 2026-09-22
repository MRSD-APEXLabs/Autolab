from glob import glob

from setuptools import find_packages, setup

package_name = 'zedx_nano_camera'

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
    zip_safe=True,
    maintainer='Arker123',
    maintainer_email='arnavkha@andrew.cmu.edu',
    description='ZED X and ZED X Nano stereo feeds from the Xavier camera hub (raw MJPEG, no ZED SDK)',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'zedx_nano_camera_node = zedx_nano_camera.camera_node:main',
            'camera_hub_node = zedx_nano_camera.hub_node:main',
        ],
    },
)
