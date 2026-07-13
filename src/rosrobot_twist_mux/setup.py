from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rosrobot_twist_mux'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
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
            'rosrobot_twist_mux = rosrobot_twist_mux.rosrobot_twist_mux:main',      # 多路分配器
            'joy2twist = joy2twist.joy2twist:main'                                  # 手柄joy话题转twist
        ],
    },
)
