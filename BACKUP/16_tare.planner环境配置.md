# [tare planner环境配置](https://github.com/shu1ong/gitblog/issues/16)

在编译时遇到了问题
```
/home/j/autonomous_exploration_development_environment/src/vehicle_simulator/src/vehicleSimulator.cpp:15:10: fatal error: gazebo_msgs/ModelState.h: 没有那个文件或目录
   15 | #include <gazebo_msgs/ModelState.h>
      |          ^~~~~~~~~~~~~~~~~~~~~~~~~~
compilation terminated.
```
解决办法：
 直接写成绝对路径即可解决