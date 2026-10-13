from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'panda_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='RPC 1 Panda Kinematics',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinematics = panda_kinematics.kinematics:main',
        ],
    },
)
