from setuptools import find_packages, setup

package_name = 'gcs_monitoring'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gcs',
    maintainer_email='gcs@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ros2_mqtt_bridge = gcs_monitoring.ros2_mqtt_bridge:main',
            'mqtt_ros2_bridge = gcs_monitoring.mqtt_ros2_bridge:main',
        ],
    },
)
