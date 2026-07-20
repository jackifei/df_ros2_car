# 小车启动流程  -- dengfei 20260706

## 运行模式 1: RViz 显示 + 键盘控制
ros2 launch rosrobot_bringup_two display_robot.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard

## 手柄控制模式: 手柄控制小车前进后退
ros2 launch learning_launch joy_motor_ctr.launch.py

### 手柄控制说明：
左摇杆：
左右用来控制前转向方向
左前按键 L 1 ：
用来停止小车，停止小车的时候左摇杆需要在0位




# 注意：修改部分记录
IMU采集周期改为50HZ，原来为200HZ









### gazebo安装 Harmonic版本
sudo apt-get update
sudo apt-get install curl lsb-release gnupg

#### gazebo安装指令
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic