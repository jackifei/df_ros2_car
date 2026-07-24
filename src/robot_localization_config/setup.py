from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_localization_config'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='793709242@qq.com',
    description='EKF robot_localization_config config for Ackermann robot at 50Hz',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={},
)
