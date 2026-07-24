from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rosrobot_top_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', ['launch/robot_twist_mux.launch.py']),
        (f'share/{package_name}/launch', ['launch/robot_control.launch.py']),
        (f'share/{package_name}/launch', ['launch/robot_test_loadurdf.launch.py']),
        (f'share/{package_name}/config', ['config/robot_control.yaml']),
        (f'share/{package_name}/config', ['config/global_params.yaml']),
        (f'share/{package_name}/config', ['config/odom_display.rviz'])
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
            # 节点入口
            'rosrobot_top_control = rosrobot_top_control.rosrobot_top_control:main'
        ],
    },
)
