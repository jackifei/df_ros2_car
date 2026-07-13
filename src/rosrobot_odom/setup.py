from setuptools import find_packages, setup

package_name = 'rosrobot_odom'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', ['launch/joystick_bridge.launch.py']),
        (f'share/{package_name}/config', ['config/vehicle_params.yaml'])
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
            'rosrobot_odom = rosrobot_odom.rosrobot_odom:main',
            'joystick_bridge_node = rosrobot_odom.joystick_bridge_node:main'
        ],
    },
)
