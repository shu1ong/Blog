# [Tare planner 实机部署的准备工作](https://github.com/shu1ong/gitblog/issues/17)

## Input Check
在官方的文档中使用的是配套的gazebo环境，确认input的方法通过rostpic进行。
将环境和planner分别启动，然后分析环境发布的topic和planner订阅的topic来确定需要进行的工作。
其中重要的话题有如下几个:
```
/state_estimation_at_scan [nav_msgs/Odometry]
/terrain_map_ext [sensor_msgs/PointCloud2]
/terrain_map [sensor_msgs/PointCloud2]
/registered_scan [sensor_msgs/PointCloud2]
/explored_areas [sensor_msgs/PointCloud2] 
```
其中`/terrain`相关的两个话题，主要处理激光点云的拼接（不同帧）和分割（根据车的距离的远近）【师兄说的】
`/state_estimation_at_scan`主要是激光和omotry的融合后的话题

---

- [ ] 小车的大小
- [ ] 激光的cropped（因为和zhang的车车激光高度不一样0.75m，可能需要调整）?
> The default sensor height is set at 0.75m above the ground in the vehicle simulator and the registered scans are cropped at the height of -0.5m and 0.25m w.r.t. the sensor.