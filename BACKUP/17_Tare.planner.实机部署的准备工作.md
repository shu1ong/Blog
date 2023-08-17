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


### log

1. 接通了scoutmini的固件和localPlanner,但出现了一定的报错

```bash
[ERROR] [1692238988.354673205]: 
Client [/scout_base_node] wants topic /cmd_vel to have datatype/md5sum [geometry_msgs/Twist/9f195f881246fdfa2798d1d3eebca84a], 
but our version has [geometry_msgs/TwistStamped/98d34b0043a2093cf9d9345ab6eef12e]. Dropping connection.
```
使用了不同的message库的原因

在pathFollower里面主要使用的是` <geometry_msgs/TwistStamped.h>`

而在scout_mini的接受的使用主要是`geometry_msgs/Twist.h`

二者之间的不同尚不清楚，但主要出现的问题为md5的哈希检验无法通过。


---

- [ ] 小车的大小
- [ ] 激光的cropped（因为和zhang的车车激光高度不一样0.75m，可能需要调整）?
> The default sensor height is set at 0.75m above the ground in the vehicle simulator and the registered scans are cropped at the height of -0.5m and 0.25m w.r.t. the sensor.
> checked: 0.25m is an absolute value