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
```c
  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce(); //这里会调用前面订阅话题时所写的句柄函数
```

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
同时在点云中添加边界信息，其中`boundaryCloud`发布的是人为划定的边界信息，而`addedObstacles`在话题中没有呈现，可能是zhagn ji早期的代码topic接口（todo：需要确认）
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
开始主循环之前，进行了一些前置的初始处理：
这些数据在后续处理中都会有比较重要的作用。
1. `pathScale`主要用于对于路径基于巡航速度的缩放，`pahtRange`还不清楚。
2. 将目标点转换到车体坐标下，进行了一个相对位置和方向的计算。
3. 限制了最大转角，在非双向行驶模式下。（能不能倒车）
4. 一共初始化了三个数组
   1. `clearPathList` ：记录筛选信息的flag，每个遮挡点++
   2. `pathPenaltyList` ：
   3. `clearPathPerGroupScore` ： 记录每个group的得分

ps`这些数组的构成类似： arr[ang1[pathNumber] ang2[pathNumber] ang3[pathNumber] ...]`
一共36个ang对应360度，然后以10度划分一组。

5. 小车的几何信息录入在 `vehicleLength`和 `vehicleWidth`中，daimeter主要是用于小车原地转圈掉头用的（oneway的情况下）
但也需要在path_generation.m中对小车的size进行更改。主要修改的参数：将serchRadius的值与diameter值相等即可。



```c
    //define pathRange
   float pathRange = adjacentRange;
   if (pathRange < minPathRange) pathRange = minPathRange;
   if (pathRangeBySpeed) pathRange = adjacentRange * joySpeed;


   bool pathFound = false;
   
   //define pathScale
   float defPathScale = pathScale;
   if (pathScaleBySpeed) pathScale = defPathScale * joySpeed;
   if (pathScale < minPathScale) pathScale = minPathScale;
   
   //goalPoint TF
   float relativeGoalDis = adjacentRange;
   //由way_point接收的数据 
   if (autonomyMode) {
     float relativeGoalX = ((goalX - vehicleX) * cosVehicleYaw + 
     (goalY - vehicleY) * sinVehicleYaw);
     float relativeGoalY = (-(goalX - vehicleX) * sinVehicleYaw + 
     (goalY - vehicleY) * cosVehicleYaw);

     relativeGoalDis = sqrt(relativeGoalX * relativeGoalX + relativeGoalY * relativeGoalY);
     joyDir = atan2(relativeGoalY, relativeGoalX) * 180 / PI;
     

        //limit the max turn around
     if (!twoWayDrive) {
       if (joyDir > 90.0) joyDir = 90.0;
       else if (joyDir < -90.0) joyDir = -90.0;
     }
   }

    //初始化三个数组
   for (int i = 0; i < 36 * pathNum; i++) {
          clearPathLiclearPathListst[i] = 0;
          pathPenaltyList[i] = 0;
        }
        for (int i = 0; i < 36 * groupNum; i++) {
          clearPathPerGroupScore[i] = 0;
        }

        float minObsAngCW = -180.0;
        float minObsAngCCW = 180.0;
        
        float diameter = sqrt(vehicleLength / 2.0 * vehicleLength / 2.0 + vehicleWidth / 2.0 * vehicleWidth / 2.0);
        float angOffset = atan2(vehicleWidth, vehicleLength) * 180.0 / PI;

```
接着进入核心函数
首先计算了每个点到车的距离，使用了`PathScale`对距离进行缩放。但实际上这里`pathScale = 1`。。。

第一个if判断了如下的条件：
1. 确定了是在近点（<adjecentRange = pathRange）
2. 排除了距离gaolPoint目标点太远的点




```c
int plannerCloudCropSize = plannerCloudCrop->points.size();
        //this loop use to filter the planner cloud
        for (int i = 0; i < plannerCloudCropSize; i++) {
          float x = plannerCloudCrop->points[i].x / pathScale; 
          float y = plannerCloudCrop->points[i].y / pathScale; 
          float h = plannerCloudCrop->points[i].intensity;
          float dis = sqrt(x * x + y * y);

if (dis < pathRange / pathScale && (dis <= (relativeGoalDis + goalClearRange) / pathScale || !pathCropByGoal) && checkObstacle) {
            for (int rotDir = 0; rotDir < 36; rotDir++) {
              float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
              float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
              if (angDiff > 180.0) {
                angDiff = 360.0 - angDiff;
              }//取劣弧
              if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
                  ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle)) {
                continue;
              }
              //return to galobal coodiantion to calculate the scaleY
              float x2 = cos(rotAng) * x + sin(rotAng) * y;
              float y2 = -sin(rotAng) * x + cos(rotAng) * y;
              //`scaleY  = y/offsetY
              float scaleY = x2 / gridVoxelOffsetX + searchRadius / gridVoxelOffsetY 
                             * (gridVoxelOffsetX - x2) / gridVoxelOffsetX;

              int indX = int((gridVoxelOffsetX + gridVoxelSize / 2 - x2) / gridVoxelSize);
              int indY = int((gridVoxelOffsetY + gridVoxelSize / 2 - y2 / scaleY) / gridVoxelSize);

              if (indX >= 0 && indX < gridVoxelNumX && indY >= 0 && indY < gridVoxelNumY) {
                int ind = gridVoxelNumY * indX + indY;
                int blockedPathByVoxelNum = correspondences[ind].size();
                for (int j = 0; j < blockedPathByVoxelNum; j++) {
                  if (h > obstacleHeightThre || !useTerrainAnalysis) {
                    clearPathList[pathNum * rotDir + correspondences[ind][j]]++;
                  } else {
                    if (pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] < h && h > groundHeightThre) {
                      pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] = h;
                    }
                  }
                }
              }
            }
          }
```