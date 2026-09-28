import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'self_balancing_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samurai',
    maintainer_email='samurai@todo.todo',
    description='Self balancing robot simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'balance_controller = self_balancing_robot.balance_controller:main',
            'keyboard_teleop = self_balancing_robot.keyboard_teleop:main',
        ],
    },
)