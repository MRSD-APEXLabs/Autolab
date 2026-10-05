from glob import glob

from setuptools import find_packages, setup

package_name = 'visual_servoing'

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
    description='Wrist-camera (ZED X Nano) visual servo grasp of a well plate on the xArm6',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'visual_servo_node = visual_servoing.servo_node:main',
        ],
    },
)
