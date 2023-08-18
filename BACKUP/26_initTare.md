# [initTare](https://github.com/shu1ong/gitblog/issues/26)

在初始化的过程中，涉及到比较复杂的回调函数的使用，故在这里对初始化的过程进行单独的分析。

还是从`SensorCoveragePlanner3D::initialize`开始,主程序的调用和回调函数的调用都呈现在这个函数中.
```c++
...//省略了一些变量赋值的过程
  execution_timer_ = nh.createTimer(ros::Duration(1.0), &SensorCoveragePlanner3D::execute, this);


  //` subscribe topic
  exploration_start_sub_    =  nh.subscribe(pp_.sub_start_exploration_topic_, 5, &SensorCoveragePlanner3D::ExplorationStartCallback, this); //` no publisher
  registered_scan_sub_      =  nh.subscribe(pp_.sub_registered_scan_topic_, 5, &SensorCoveragePlanner3D::RegisteredScanCallback, this);
  terrain_map_sub_          =  nh.subscribe(pp_.sub_terrain_map_topic_, 5, &SensorCoveragePlanner3D::TerrainMapCallback, this);
  terrain_map_ext_sub_      =  nh.subscribe(pp_.sub_terrain_map_ext_topic_, 5, &SensorCoveragePlanner3D::TerrainMapExtCallback, this);
  state_estimation_sub_     =  nh.subscribe(pp_.sub_state_estimation_topic_, 5, &SensorCoveragePlanner3D::StateEstimationCallback, this);
  coverage_boundary_sub_    =  nh.subscribe(pp_.sub_coverage_boundary_topic_, 1, &SensorCoveragePlanner3D::CoverageBoundaryCallback, this);
  viewpoint_boundary_sub_   =  nh.subscribe(pp_.sub_viewpoint_boundary_topic_, 1, &SensorCoveragePlanner3D::ViewPointBoundaryCallback, this);
  nogo_boundary_sub_        =  nh.subscribe(pp_.sub_nogo_boundary_topic_, 1, &SensorCoveragePlanner3D::NogoBoundaryCallback, this);
...//省略了话题的发布
  return true;
}

```
从上面的程序可以看出首先调用的时execute的函数返回了之后 在ros::spin()中会进行各个订阅话题回调函数的执行.

从`execute`开始
```c++
void SensorCoveragePlanner3D::execute(const ros::TimerEvent&){
  if (!pp_.kAutoStart && !start_exploration_)
  {
    ROS_INFO("Waiting for start signal");
    return;
  }
  ...//省略了一些flag的defien
  if (!initialized_) {
    SendInitialWaypoint();
    start_time_ = ros::Time::now();
    global_direction_switch_time_ = ros::Time::now();
    return;
  }
```

当程序运行到此处的时候初始化并没有完成,`intialized_ = 0`所以进入了条件判断.

并执行函数`SendInitialWaypoint()`

然后开启了两个时间的记录,具体功能后面看.

函数`SendInitialWaypoint()`的**主要作用**是

在小车的前方初始化了一个waypoint并把这个waypoint转换到全局坐标中发布到`\way_point`的topic当中.
```c++
void SensorCoveragePlanner3D::SendInitialWaypoint() {
  // lx ly only determine the dir of the start which will update later
  double lx = 12.0;
  double ly = 0.0;
  //TF to body coordination
  double dx = cos(pd_.robot_yaw_) * lx - sin(pd_.robot_yaw_) * ly;
  double dy = sin(pd_.robot_yaw_) * lx + cos(pd_.robot_yaw_) * ly;
  geometry_msgs::PointStamped waypoint;
  waypoint.header.frame_id = "map";
  waypoint.header.stamp = ros::Time::now();
  waypoint.point.x = pd_.robot_position_.x + dx;
  waypoint.point.y = pd_.robot_position_.y + dy;
  waypoint.point.z = pd_.robot_position_.z;
  waypoint_pub_.publish(waypoint);
}
```

在这里if判断就return掉了,程序就回到了initialize中,于是就该执行之后的订阅话题操作中的回调函数了.

一个个看:

第一个回调函数`ExplorationStartCallback`,主要用于接受开始信号的.
```c++
void SensorCoveragePlanner3D::ExplorationStartCallback(const std_msgs::Bool::ConstPtr& start_msg)
{
  if (start_msg->data)
  {
    start_exploration_ = true;
  }
}
```
第二个函数为`RegisteredScanCallback`,在初始化中被直接跳过了.
```c++
void SensorCoveragePlanner3D::RegisteredScanCallback(const sensor_msgs::PointCloud2ConstPtr& registered_scan_msg)
{
  if (!initialized_) {
    return;
  }
```



