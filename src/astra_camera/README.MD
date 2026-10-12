ros2 jazzy版本

编译指令

> colcon build --packages-select astra_camera_msgs --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

需要先编译msgs
然后编译相机驱动

> 
> colcon build --packages-select astra_camera --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
> 
> 
> 


如果使用RVIZ2实现点云，请将fixed frame 修改为camera_link
