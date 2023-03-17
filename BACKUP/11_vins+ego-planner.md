# [vins+ego-planner](https://github.com/shu1ong/gitblog/issues/11)

启动gazebo仿真
```
roslaunch px4 indoor1.launch
```
启动vins（我把vins的rviz注释掉了）
```
cd ~/catkin_ws
bash scripts/xtdrone_run_vio.sh
```
话题类型转化：vins发布位置
```
cd ~/XTDrone/sensing/slam/vio
python vins_transfer.py iris 0
```
构建键盘通讯
```
cd ~/XTDrone/communication
python multirotor_communication.py iris 0 
```
唤起键盘控制菜单
```
cd ~/XTDrone/control/keyboard
python multirotor_keyboard_control.py iris 1 vel
```

ego-planner 启动
转换坐标
```
cd ~/XTDrone/motion_planning/3d
python ego_transfer.py iris 0
```
启动rviz
```
cd ~/XTDrone/motion_planning/3d
rviz -d ego_rviz.rviz
```


启动ego_planner
```
cd ~/XTDrone/motion_planning/3d
roslaunch ego_planner single_uav.launch
```


---

起飞后切hover会有点奇怪，会有一个抬头的动作，不知道是不是加速度设置过大的原因