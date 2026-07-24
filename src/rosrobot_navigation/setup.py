from setuptools import find_packages, setup
from glob import glob
import os


package_name = 'rosrobot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (f'share/{package_name}/launch', ['launch/nav_control.launch.py']),
        (f'share/{package_name}/config', ['config/nav2_params.yaml']),
        (f'share/{package_name}/maps', ['maps/map_edited.yaml']),
        (f'share/{package_name}/maps', ['maps/map_edited.pgm'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dengfei',
    maintainer_email='793709242@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rosrobot_nav = rosrobot_navigation.rosrobot_nav:main'
        ],
    },
)
