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