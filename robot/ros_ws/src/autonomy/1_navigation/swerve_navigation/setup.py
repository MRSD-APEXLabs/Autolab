from setuptools import find_packages, setup

package_name = 'swerve_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='labx',
    maintainer_email='arnavkha@andrew.cmu.edu',
    description='Navigation nodes for the MK5 swerve base: hardware bridge, lidar pipeline, '
                'teleop, cmd_vel mux, goal helpers and the /nav topic API.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'swerve_bridge = swerve_navigation.swerve_bridge:main',
            'joy_teleop = swerve_navigation.joy_teleop:main',
            'cmd_vel_mux = swerve_navigation.cmd_vel_mux:main',
            'cloud_to_scan = swerve_navigation.cloud_to_scan:main',
            'point_to_goal = swerve_navigation.point_to_goal:main',
            'location_markers = swerve_navigation.location_markers:main',
            'nav_api = swerve_navigation.nav_api:main',
        ],
    },
)
