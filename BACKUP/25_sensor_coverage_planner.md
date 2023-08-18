# [sensor_coverage_planner](https://github.com/shu1ong/gitblog/issues/25)

## 程序节点分析与主程序入口逻辑

Tare的程序分析从launch文件开始。这里选择最常用的indoor.launch.

其中比较重要的有两段，一段是对`explore.launch`的调用，一段是对`navigationBoundary`pkg的调用。

前者是launch文件的套用，一般用来满足不同参数的配置.后者是用于进行探索中边界的定义.在demo演示中一般用不上.

这是explore.launch中的重要部分,指明了节点的发布.
```xml
    <node pkg="tare_planner" type="tare_planner_node" name="tare_planner_node" output="screen" ns="sensor_coverage_planner">
        <rosparam command="load" file="$(find tare_planner)/config/$(arg scenario).yaml" />
    </node>
```

该节点对应的cpp文件在src中去寻找,对应关系描述在cmakelists中
```c
add_executable(navigationBoundary src/navigation_boundary_publisher/navigationBoundary.cpp)
target_link_libraries(navigationBoundary ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(tare_planner_node src/tare_planner_node/tare_planner_node.cpp)
add_dependencies(tare_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(tare_planner_node ${catkin_LIBRARIES} sensor_coverage_planner_ground)
```
最后确定到主程序的位置在`tare_planner_node.cpp`中，

在初始化节点之后,进行了类的实体化,并取名为`tare_planner`, 并传入了ros节点的句柄

然后运行了ros::spin()进行消息的通讯

```c
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tare_planner_node");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");

  sensor_coverage_planner_3d_ns::SensorCoveragePlanner3D tare_planner(node_handle, private_node_handle);

  ros::spin();
  return 0;
}
```
所以去找`sensor_coverage_planner_3d_ns` namesapce下定义的class `SensorCoveragePlanner3D`

.h的头文件里可以看到该类的声明其中包括了同名的构造函数,这一构造函数在类实体化时会自动调用(根据传入的参数).
```c
class SensorCoveragePlanner3D
{
public:
  explicit SensorCoveragePlanner3D(ros::NodeHandle& nh, ros::NodeHandle& nh_p);
  bool initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_p);
  void execute(const ros::TimerEvent&);
  ~SensorCoveragePlanner3D() = default;

...
}
```
而该函数的定义则是在.cpp的文件中完成的,在定义的同时进行了一些参数的初始化.他的主要作用是拉起对于`initialize`的调用.

```c
SensorCoveragePlanner3D::SensorCoveragePlanner3D(ros::NodeHandle& nh, ros::NodeHandle& nh_p)
  : keypose_cloud_update_(false)
  , initialized_(false)
  , lookahead_point_update_(false)
  , relocation_(false)
  , start_exploration_(false)
  , exploration_finished_(false)
  , near_home_(false)
  , at_home_(false)
  , stopped_(false)
  , test_point_update_(false)
  , viewpoint_ind_update_(false)
  , step_(false)
  , use_momentum_(false)
  , lookahead_point_in_line_of_sight_(true)
  , registered_cloud_count_(0)
  , keypose_count_(0)
  , direction_change_count_(0)
  , direction_no_change_count_(0)
  , momentum_activation_count_(0)
{
  initialize(nh, nh_p);
  PrintExplorationStatus("Exploration Started", false);
}
```
## `initialize`函数的作用

首先是参数的读取,主要是一些点云相关的变量以及其匹配对应的话题和一些配置用的bool变量
```c
bool SensorCoveragePlanner3D::initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_p)
{
  if (!pp_.ReadParameters(nh_p)){
    ROS_ERROR("Read parameters failed");
    return false;
  }
```
另外的一个初始化程序. 也是一堆重载的点云文件，暂时跳过.不过里面涉及到了车体的位置初始化.
```c
  pd_.Initialize(nh, nh_p);

```
关于位置初始化
```c
void PlannerData::Initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_p){
  ...
  robot_position_.x = 0;
  robot_position_.y = 0;
  robot_position_.z = 0;

  last_robot_position_ = robot_position_;
}
```
然后这两句也整不明白
```c
  pd_.keypose_graph_->SetAllowVerticalEdge(false);
  lidar_model_ns::LiDARModel::setCloudDWZResol(pd_.planning_env_->GetPlannerCloudResolution());
  ```
  
  然后终于到了主角出场,这里的timer函数间隔为1s,每隔1s执行一次`excute`这个函数
  ```c
  //` every second call the execute function(main function) 
  execution_timer_ = nh.createTimer(ros::Duration(1.0), &SensorCoveragePlanner3D::execute, this);
  ```
  接着进行相关话题的订阅,并执行相应的句柄函数.
  ```c
  exploration_start_sub_    =  nh.subscribe(pp_.sub_start_exploration_topic_, 5, &SensorCoveragePlanner3D::ExplorationStartCallback, this); //` no publisher
  registered_scan_sub_      =  nh.subscribe(pp_.sub_registered_scan_topic_, 5, &SensorCoveragePlanner3D::RegisteredScanCallback, this);
  ...
  nogo_boundary_sub_        =  nh.subscribe(pp_.sub_nogo_boundary_topic_, 1, &SensorCoveragePlanner3D::NogoBoundaryCallback, this);
  ```
  这里是关于话题发布的部分
  ```c
  global_path_full_publisher_                = nh.advertise<nav_msgs::Path>("global_path_full", 1);
  global_path_publisher_                     = nh.advertise<nav_msgs::Path>("global_path", 1);
  ...
  momentum_activation_count_pub_             = nh.advertise<std_msgs::Int32>(pp_.pub_momentum_activation_count_topic_, 2);
  ...
  return true;
}

```
## 主函数excute

这里的start信号可以用topic的方式进行发布
```c
void SensorCoveragePlanner3D::execute(const ros::TimerEvent&)
{
  if (!pp_.kAutoStart && !start_exploration_)
  {
    ROS_INFO("Waiting for start signal");
    return;
  }
```
一些bool flag
```c
  Timer overall_processing_timer("overall processing");
  update_representation_runtime_ = 0;
  local_viewpoint_sampling_runtime_ = 0;
  local_path_finding_runtime_ = 0;
  global_planning_runtime_ = 0;
  trajectory_optimization_runtime_ = 0;
  overall_runtime_ = 0;
```
当没有进行初始化的时候进行初始化
```c

  if (!initialized_)
  {
    SendInitialWaypoint();
    start_time_ = ros::Time::now();
    global_direction_switch_time_ = ros::Time::now();
    return;
  }
```
主要是对waypoint的初始化
```c
void SensorCoveragePlanner3D::SendInitialWaypoint()
{
  // lx ly only determine the dir of the start which will update later
  double lx = 12.0;
  double ly = 0.0;
  //从车体坐标系到世界坐标系下,waypoint初始化始终是在小车的行驶方向的正前方,需要投影到世界坐标下去计算其绝对坐标
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

```c
  overall_processing_timer.Start();
  if (keypose_cloud_update_)
  {
    keypose_cloud_update_ = false;

    CountDirectionChange();

    misc_utils_ns::Timer update_representation_timer("update representation");
    update_representation_timer.Start();

    // Update grid world
    UpdateGlobalRepresentation();

    int viewpoint_candidate_count = UpdateViewPoints();
    if (viewpoint_candidate_count == 0)
    {
      ROS_WARN("Cannot get candidate viewpoints, skipping this round");
      return;
    }

    UpdateKeyposeGraph();

    int uncovered_point_num = 0;
    int uncovered_frontier_point_num = 0;
    if (!exploration_finished_ || !pp_.kNoExplorationReturnHome)
    {
      UpdateViewPointCoverage();
      UpdateCoveredAreas(uncovered_point_num, uncovered_frontier_point_num);
    }
    else
    {
      pd_.viewpoint_manager_->ResetViewPointCoverage();
    }

    update_representation_timer.Stop(false);
    update_representation_runtime_ += update_representation_timer.GetDuration("ms");

    // Global TSP
    std::vector<int> global_cell_tsp_order;
    exploration_path_ns::ExplorationPath global_path;
    GlobalPlanning(global_cell_tsp_order, global_path);

    // Local TSP
    exploration_path_ns::ExplorationPath local_path;
    LocalPlanning(uncovered_point_num, uncovered_frontier_point_num, global_path, local_path);

    near_home_ = GetRobotToHomeDistance() < pp_.kRushHomeDist;
    at_home_ = GetRobotToHomeDistance() < pp_.kAtHomeDistThreshold;

    if (pd_.grid_world_->IsReturningHome() && pd_.local_coverage_planner_->IsLocalCoverageComplete() &&
        (ros::Time::now() - start_time_).toSec() > 5)
    {
      if (!exploration_finished_)
      {
        PrintExplorationStatus("Exploration completed, returning home", false);
      }
      exploration_finished_ = true;
    }

    if (exploration_finished_ && at_home_ && !stopped_)
    {
      PrintExplorationStatus("Return home completed", false);
      stopped_ = true;
    }

    pd_.exploration_path_ = ConcatenateGlobalLocalPath(global_path, local_path);

    PublishExplorationState();

    lookahead_point_update_ = GetLookAheadPoint(pd_.exploration_path_, global_path, pd_.lookahead_point_);
    PublishWaypoint();

    overall_processing_timer.Stop(false);
    overall_runtime_ = overall_processing_timer.GetDuration("ms");

    pd_.visualizer_->GetGlobalSubspaceMarker(pd_.grid_world_, global_cell_tsp_order);
    Eigen::Vector3d viewpoint_origin = pd_.viewpoint_manager_->GetOrigin();
    pd_.visualizer_->GetLocalPlanningHorizonMarker(viewpoint_origin.x(), viewpoint_origin.y(), pd_.robot_position_.z);
    pd_.visualizer_->PublishMarkers();

    PublishLocalPlanningVisualization(local_path);
    PublishGlobalPlanningVisualization(global_path, local_path);
    PublishRuntime();
  }
}
}  // namespace sensor_coverage_planner_3d_ns

```

