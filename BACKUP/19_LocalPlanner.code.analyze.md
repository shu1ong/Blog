# [LocalPlanner code analyze](https://github.com/shu1ong/gitblog/issues/19)

# inti
### 直接进主函数，节点的初始化

```c++
int main(int argc, char** argv)
{
  ros::init(argc, argv, "localPlanner");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");
```

### 然后读取了一些参数，这些参数主要在launch文件中进行配置

```c++
...
  nhPrivate.getParam("pathFolder", pathFolder);
  nhPrivate.getParam("vehicleLength", vehicleLength);
  nhPrivate.getParam("vehicleWidth", vehicleWidth);
  nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
  nhPrivate.getParam("sensorOffsetY", sensorOffsetY);
  nhPrivate.getParam("twoWayDrive", twoWayDrive);
  nhPrivate.getParam("laserVoxelSize", laserVoxelSize);
  nhPrivate.getParam("terrainVoxelSize", terrainVoxelSize);
  nhPrivate.getParam("useTerrainAnalysis", useTerrainAnalysis);
  nhPrivate.getParam("checkObstacle", checkObstacle);
  nhPrivate.getParam("checkRotObstacle", checkRotObstacle);
  nhPrivate.getParam("adjacentRange", adjacentRange);
  nhPrivate.getParam("obstacleHeightThre", obstacleHeightThre);
...
```
### 节点订阅，消息定义
```c++
  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry> ("/state_estimation", 5, odometryHandler);
  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> ("/registered_scan", 5, laserCloudHandler);
  ros::Subscriber subTerrainCloud = nh.subscribe<sensor_msgs::PointCloud2> ("/terrain_map", 5, terrainCloudHandler);
  ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy> ("/joy", 5, joystickHandler);
  ros::Subscriber subGoal = nh.subscribe<geometry_msgs::PointStamped> ("/way_point", 5, goalHandler);
  ros::Subscriber subSpeed = nh.subscribe<std_msgs::Float32> ("/speed", 5, speedHandler);
  ros::Subscriber subBoundary = nh.subscribe<geometry_msgs::PolygonStamped> ("/navigation_boundary", 5, boundaryHandler);
  ros::Subscriber subAddedObstacles = nh.subscribe<sensor_msgs::PointCloud2> ("/added_obstacles", 5, addedObstaclesHandler);
  ros::Subscriber subCheckObstacle = nh.subscribe<std_msgs::Bool> ("/check_obstacle", 5, checkObstacleHandler);
  ros::Publisher pubPath = nh.advertise<nav_msgs::Path> ("/path", 5);
  nav_msgs::Path path;
```
关于话题的订阅有几点值得说明的：
1. /registered_scan 和 /terrain_map 应该都是激光扫描出来的地图信息，都可以作为input，这里应该是给了两种不同的接口。而在simulator中使用的是/terrain_map。
2. /state_estimation是simulator中自带的状态估计怎么和imu接上尚且不知
3. /way_point为TarePlanenr的output 发布话题，在没有开启palnner的情况下，可以通过rviz进行way_point的发布
4. /joy节点是引用的ps3手柄库，在joy路径下也有ps4的库，ps3的手柄库和xbox通用。连接之后注意输入应该是Dinput模式，系统才能读到手柄的信号输入
5. 关于手柄的信号输入有这么几点需要注意,在rostopic中输出的数据中axes数组中会有0值占位，但在joy->axes[]的调用中没有。注意对应关系。目前测得的对应关系如下(主要键位)：
```
axes[0 1 2 3 4 5] = [lh lv rh rv rt lt]
lh : left horrizon
lv : left vertical
lt : left trigger
```
### path文件的读取
```c++
printf ("\nReading path files.\n");

  if (autonomyMode) {
    joySpeed = autonomySpeed / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }

  for (int i = 0; i < laserCloudStackNum; i++) {
    laserCloudStack[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  for (int i = 0; i < groupNum; i++) {
    startPaths[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
  }
  #if PLOTPATHSET == 1
  for (int i = 0; i < pathNum; i++) {
    paths[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  #endif
  for (int i = 0; i < gridVoxelNum; i++) {
    correspondences[i].resize(0);
  }

  laserDwzFilter.setLeafSize(laserVoxelSize, laserVoxelSize, laserVoxelSize);
  terrainDwzFilter.setLeafSize(terrainVoxelSize, terrainVoxelSize, terrainVoxelSize);

  readStartPaths();
  #if PLOTPATHSET == 1
  readPaths();
  #endif
  readPathList();
```
这里path的读取一共有三个函数，分别读取了`startPaths.ply``Paths.ply``Pathlist.ply`三个文件。
这三个文件都是由文件`path_generation.m`生成。一起保存在`../path`路径下。
文件中保存了path的前向路径生成和体素网格邻近表，用作碰撞检测和路径筛选。可以查阅另一篇博客进行参考（todo）。
# main
之前的内容都属于初始化的内容，loop从这里开始

### 对于激光点云数据进行处理
如果是一帧的激光数据则与之前的激光数据进行叠加，而如果是terrainmap（已经处理好的数据？）则直接对plannerCloud进行更新
```c++
    if (newLaserCloud || newTerrainCloud) {
      if (newLaserCloud) {
        newLaserCloud = false;

        laserCloudStack[laserCloudCount]->clear();
        *laserCloudStack[laserCloudCount] = *laserCloudDwz;
        laserCloudCount = (laserCloudCount + 1) % laserCloudStackNum;

        plannerCloud->clear();
        for (int i = 0; i < laserCloudStackNum; i++) {
          *plannerCloud += *laserCloudStack[i];
        }
      }

      if (newTerrainCloud) {
        newTerrainCloud = false;

        plannerCloud->clear();
        *plannerCloud = *terrainCloudDwz;
      }

```

然后对点云做了近邻裁剪（在车体坐标下）
```c
pcl::PointXYZI point;
      plannerCloudCrop->clear();
      int plannerCloudSize = plannerCloud->points.size();
      for (int i = 0; i < plannerCloudSize; i++) {
        float pointX1 = plannerCloud->points[i].x - vehicleX;
        float pointY1 = plannerCloud->points[i].y - vehicleY;
        float pointZ1 = plannerCloud->points[i].z - vehicleZ;

        point.x = pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
        point.y = -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
        point.z = pointZ1;
        point.intensity = plannerCloud->points[i].intensity;

        float dis = sqrt(point.x * point.x + point.y * point.y);
        if (dis < adjacentRange && ((point.z > minRelZ && point.z < maxRelZ) || useTerrainAnalysis)) {
          plannerCloudCrop->push_back(point);
        }
      }
```
同时在点云中添加边界信息
```c
      int boundaryCloudSize = boundaryCloud->points.size();
      for (int i = 0; i < boundaryCloudSize; i++) {
        point.x = ((boundaryCloud->points[i].x - vehicleX) * cosVehicleYaw + (boundaryCloud->points[i].y - vehicleY) * sinVehicleYaw);
        point.y = (-(boundaryCloud->points[i].x - vehicleX) * sinVehicleYaw + (boundaryCloud->points[i].y - vehicleY) * cosVehicleYaw);
        point.z = boundaryCloud->points[i].z;
        point.intensity = boundaryCloud->points[i].intensity;

        float dis = sqrt(point.x * point.x + point.y * point.y);
        if (dis < adjacentRange) {
          plannerCloudCrop->push_back(point);
        }
      }



      int addedObstaclesSize = addedObstacles->points.size();
      for (int i = 0; i < addedObstaclesSize; i++) {
        point.x = ((addedObstacles->points[i].x - vehicleX) * cosVehicleYaw 
                + (addedObstacles->points[i].y - vehicleY) * sinVehicleYaw);
        point.y = (-(addedObstacles->points[i].x - vehicleX) * sinVehicleYaw 
                + (addedObstacles->points[i].y - vehicleY) * cosVehicleYaw);
        point.z = addedObstacles->points[i].z;
        point.intensity = addedObstacles->points[i].intensity;

        float dis = sqrt(point.x * point.x + point.y * point.y);
        if (dis < adjacentRange) {
          plannerCloudCrop->push_back(point);
        }
      }
      ```