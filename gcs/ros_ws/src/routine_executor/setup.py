from setuptools import find_packages, setup

package_name = 'routine_executor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, ['scripts/run_routine.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gcs',
    maintainer_email='gcs@todo.todo',
    description='GCS-side routine executor',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'routine_executor_node = routine_executor.node:main',
        ],
    },
)
