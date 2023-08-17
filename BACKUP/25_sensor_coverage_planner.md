# [sensor_coverage_planner](https://github.com/shu1ong/gitblog/issues/25)

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

这些配置的信息存在.h的头文件里

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

