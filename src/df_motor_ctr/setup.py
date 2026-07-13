from setuptools import find_packages, setup

package_name = 'df_motor_ctr'

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
            'get_topic_joy = df_motor_ctr.get_topic_joy:main',
            'motor_ctr = df_motor_ctr.motor_control:main',   # 电机控制
            'wheel_dir = df_motor_ctr.wheel_dir_pwm:main'    # 转向控制
        ],
    },
)
