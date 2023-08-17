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
  
  ```c
  //` subscribe topic
  exploration_start_sub_    =  nh.subscribe(pp_.sub_start_exploration_topic_, 5, &SensorCoveragePlanner3D::ExplorationStartCallback, this); //` no publisher
  registered_scan_sub_      =  nh.subscribe(pp_.sub_registered_scan_topic_, 5, &SensorCoveragePlanner3D::RegisteredScanCallback, this);
  terrain_map_sub_          =  nh.subscribe(pp_.sub_terrain_map_topic_, 5, &SensorCoveragePlanner3D::TerrainMapCallback, this);
  terrain_map_ext_sub_      =  nh.subscribe(pp_.sub_terrain_map_ext_topic_, 5, &SensorCoveragePlanner3D::TerrainMapExtCallback, this);
  state_estimation_sub_     =  nh.subscribe(pp_.sub_state_estimation_topic_, 5, &SensorCoveragePlanner3D::StateEstimationCallback, this);
  coverage_boundary_sub_    =  nh.subscribe(pp_.sub_coverage_boundary_topic_, 1, &SensorCoveragePlanner3D::CoverageBoundaryCallback, this);
  viewpoint_boundary_sub_   =  nh.subscribe(pp_.sub_viewpoint_boundary_topic_, 1, &SensorCoveragePlanner3D::ViewPointBoundaryCallback, this);
  nogo_boundary_sub_        =  nh.subscribe(pp_.sub_nogo_boundary_topic_, 1, &SensorCoveragePlanner3D::NogoBoundaryCallback, this);
  //` establish publisher
  global_path_full_publisher_                = nh.advertise<nav_msgs::Path>("global_path_full", 1);
  global_path_publisher_                     = nh.advertise<nav_msgs::Path>("global_path", 1);
  old_global_path_publisher_                 = nh.advertise<nav_msgs::Path>("old_global_path", 1);
  to_nearest_global_subspace_path_publisher_ = nh.advertise<nav_msgs::Path>("to_nearest_global_subspace_path", 1);
  local_tsp_path_publisher_                  = nh.advertise<nav_msgs::Path>("local_path", 1);
  exploration_path_publisher_                = nh.advertise<nav_msgs::Path>("exploration_path", 1);
  waypoint_pub_                              = nh.advertise<geometry_msgs::PointStamped>(pp_.pub_waypoint_topic_, 2);
  exploration_finish_pub_                    = nh.advertise<std_msgs::Bool>(pp_.pub_exploration_finish_topic_, 2);
  runtime_breakdown_pub_                     = nh.advertise<std_msgs::Int32MultiArray>(pp_.pub_runtime_breakdown_topic_, 2);
  runtime_pub_                               = nh.advertise<std_msgs::Float32>(pp_.pub_runtime_topic_, 2);
  momentum_activation_count_pub_             = nh.advertise<std_msgs::Int32>(pp_.pub_momentum_activation_count_topic_, 2);
  // Debug
  pointcloud_manager_neighbor_cells_origin_pub_ = nh.advertise<geometry_msgs::PointStamped>("pointcloud_manager_neighbor_cells_origin", 1);
  return true;
}

```



