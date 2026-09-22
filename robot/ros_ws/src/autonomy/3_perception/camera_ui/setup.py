from glob import glob

from setuptools import find_packages, setup

package_name = 'camera_ui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['web/index.html']},
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
    description='Browser UI to switch between the ZED X and ZED X Nano and view RGB and depth',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'camera_ui_node = camera_ui.ui_node:main',
        ],
    },
)
