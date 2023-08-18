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

**第一个回调函数**`ExplorationStartCallback`,主要用于接受开始信号的.
```c++
void SensorCoveragePlanner3D::ExplorationStartCallback(const std_msgs::Bool::ConstPtr& start_msg)
{
  if (start_msg->data)
  {
    start_exploration_ = true;
  }
}
```
**第二个函数**为`RegisteredScanCallback`,在初始化中被直接跳过了.
```c++
void SensorCoveragePlanner3D::RegisteredScanCallback(const sensor_msgs::PointCloud2ConstPtr& registered_scan_msg)
{
  if (!initialized_) {
    return;
  }
```
**第三个**`TerrainMapCallback`,对读取到的terrainmap做了简单的类型转化之后进行了过滤:用`intensity`过滤掉了大于`kTerrainCollisionThreshold`的值.

返回了的点云文件叫做`terrain_collision_cloud_`
```C++
void SensorCoveragePlanner3D::TerrainMapCallback(const sensor_msgs::PointCloud2ConstPtr& terrain_map_msg){
  if (pp_.kCheckTerrainCollision){//const true 
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_map_tmp(new pcl::PointCloud<pcl::PointXYZI>());
    //这里是把前者的信息转换为后者
    pcl::fromROSMsg<pcl::PointXYZI>(*terrain_map_msg, *terrain_map_tmp);
    pd_.terrain_collision_cloud_->cloud_->clear();
    for (auto& point : terrain_map_tmp->points){
      if (point.intensity > pp_.kTerrainCollisionThreshold) {
        pd_.terrain_collision_cloud_->cloud_->points.push_back(point);
      }
    }
  }
}
```
**第四个函数**`TerrainMapExtCallback`,与上一个函数基本相同,这个用来接受广域terrainmap的,用不上暂时略过.

**第五个函数**`StateEstimationCallback`,从话题`state_estimation`中获取当前小车的位姿信息.**将`initialized_`赋值为true**
```c++
void SensorCoveragePlanner3D::StateEstimationCallback(const nav_msgs::Odometry::ConstPtr& state_estimation_msg) {
  pd_.robot_position_ = state_estimation_msg->pose.pose.position;//这里初始化为0,此刻被赋传感器的真值(一般是经过slam计算得到后的值)
  // Todo: use a boolean
  if (std::abs(pd_.initial_position_.x()) < 0.01 && std::abs(pd_.initial_position_.y()) < 0.01 && std::abs(pd_.initial_position_.z()) < 0.01) {
    pd_.initial_position_.x() = pd_.robot_position_.x;
    pd_.initial_position_.y() = pd_.robot_position_.y;
    pd_.initial_position_.z() = pd_.robot_position_.z;
  }
  
  //将四元数转化为RPY的格式
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geo_quat = state_estimation_msg->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w)).getRPY(roll, pitch, yaw);

  pd_.robot_yaw_ = yaw;
  //小车的前进判断
  if (state_estimation_msg->twist.twist.linear.x > 0.4) {
    pd_.moving_forward_ = true;
  }
  else if (state_estimation_msg->twist.twist.linear.x < -0.4) {
    pd_.moving_forward_ = false;
  }
  initialized_ = true;
}
```

**第六个函数**`\coverage_boundary`,谜之操作看不懂

```C++
void SensorCoveragePlanner3D::CoverageBoundaryCallback(const geometry_msgs::PolygonStampedConstPtr& polygon_msg)
{
  pd_.planning_env_->UpdateCoverageBoundary((*polygon_msg).polygon);
}
..................................
inline void UpdateCoverageBoundary(const geometry_msgs::Polygon& polygon)
  {
    coverage_boundary_ = polygon;
  }
```




