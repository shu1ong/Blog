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
接着进入核心函数，主要功能为点云的碰撞检测：

程序会遍历每个点，与环路径（360）进行匹配。匹配的方式通过距离距离计算进行，对应到文件中的体素索引。

将邻近路径进行标记。标记个数大于阈值时将path进行虑除。同时在遍历时加入了一定的规则去减小需要遍历的方向，加快计算。

下面是详细的程序解析。

首先计算了每个点到车的距离，使用了`PathScale`对距离进行缩放。但实际上这里`pathScale = 1`。。。

第一个if判断了如下的条件：
1. 确定了是在近点（<adjecentRange = pathRange）
2. 排除了距离gaolPoint目标点太远的点（dis = 0.5）
   
```c
int plannerCloudCropSize = plannerCloudCrop->points.size();
        //this loop use to filter the planner cloud
        for (int i = 0; i < plannerCloudCropSize; i++) {
          float x = plannerCloudCrop->points[i].x / pathScale; 
          float y = plannerCloudCrop->points[i].y / pathScale; 
          float h = plannerCloudCrop->points[i].intensity;
          float dis = sqrt(x * x + y * y);

if (dis < pathRange / pathScale && (dis <= (relativeGoalDis + goalClearRange) / pathScale || !pathCropByGoal) && checkObstacle) {
```
这里直接进行的是方向上的划分，将path接在了36个方向上，形成了一个包围小车的大圆。
然后得对每个方向进行遍历。

注意这里所有的计算都是基于车体坐标系下进行的。

然后计算了当其目标点与遍历方向之间的夹角。
```c
            for (int rotDir = 0; rotDir < 36; rotDir++) {
              float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
              float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
              if (angDiff > 180.0) {
                angDiff = 360.0 - angDiff;
              }//取劣弧
```
如果说不想进入continue则需要使得if判断为假，那么所有的||运算的元素都必须为false

则有

`(angDiff > dirThre && !dirToVehicle) = false`

又有`dirToVehicle = false`，所以必须有`angDiff > dirThre == false`
取其补集则有：`angDiff <= dirThre`此时angDiff所得出的方向均为背离目标点的方向。

```c
              if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
                  ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle)) {
                continue;
              }
```
把这些方向上的点逆变换回车体坐标系下，（纯逆旋转）
用x2的值可以求出scaleY,其作用是用来求解indY: 

`scaleY =  y / (offsetY - voxelSize * indY)`

因为体素网格不是沿x轴进行均匀分布的。但每一个indX所对应的网格数量是相同的。


```c
              //return to galobal coodiantion to calculate the scaleY
              float x2 = cos(rotAng) * x + sin(rotAng) * y;
              float y2 = -sin(rotAng) * x + cos(rotAng) * y;
              //`scaleY  = y/offsetY
              float scaleY = x2 / gridVoxelOffsetX + searchRadius / gridVoxelOffsetY 
                             * (gridVoxelOffsetX - x2) / gridVoxelOffsetX;

              int indX = int((gridVoxelOffsetX + gridVoxelSize / 2 - x2) / gridVoxelSize);
              int indY = int((gridVoxelOffsetY + gridVoxelSize / 2 - y2 / scaleY) / gridVoxelSize);
```
检验体素网格是否在路径范围内，如果在的话，遍历当前体素网格附近所有的路径，并将相关标签记录杂clearPathList中。（被遮挡了一次）

这里有对障碍点的高度判断，需要大于障碍物高度阈值的点才会被识别为障碍物。（terrain analysis）

而如果该点在terrain analysis判断后依旧能够通过，则将它的高度值加入到`PathPenaltyList`中，为后续评分提供依据。（可以经过但不会优先选择的点）

```c
              if (indX >= 0 && indX < gridVoxelNumX && indY >= 0 && indY < gridVoxelNumY) {
                int ind = gridVoxelNumY * indX + indY;
                //每一列的体素网格数是一个定值


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
需要原地旋转时应该满足的条件：
1. 点在小车的外切圆之内
2. 该点不在小车内
3. 满足terrain analysis

```c
if (dis < diameter / pathScale && (fabs(x) > vehicleLength / pathScale / 2.0 || fabs(y) > vehicleWidth / pathScale / 2.0) && (h > obstacleHeightThre || !useTerrainAnalysis) && checkRotObstacle) {
            //condition 1: dis < diameter / pathScale                                                                     ---->satify the situation is that the point is too close to the vehicle
            //conditoin 2: fabs(x) > vehicleLength / pathScale / 2.0 || fabs(y) > vehicleWidth / pathScale / 2.0          ----->but the point cant be in the vehicle
            //condition 3: h > obstacleHeightThre || !useTerrainAnalysis                                                  ------>meanwile it should satify the hight check
            //conditino 4: checkRotObstacle                                                                               ----->mode configuration
```
在逆时针方向 这里的Obs应该是observasion的意思正好满足观察到小车的最小旋转角度

分别分为两个方向，angObs的正向为逆时针

而这里的`minObsAngCCW`和`minObsAngCW` 应该是正好互补的(180)
```c
            float angObs = atan2(y, x) * 180.0 / PI;
            if (angObs > 0) {
              if (minObsAngCCW > angObs - angOffset) minObsAngCCW = angObs - angOffset;
              if (minObsAngCW < angObs + angOffset - 180.0) minObsAngCW = angObs + angOffset - 180.0;
            } else {
              //在逆时针方向
              if (minObsAngCW < angObs + angOffset) minObsAngCW = angObs + angOffset;
              if (minObsAngCCW > 180.0 + angObs - angOffset) minObsAngCCW = 180.0 + angObs - angOffset;
            }
          }
        }
```

对所有的path进行遍历，其中rotDir的取值为0～35
```c
for (int i = 0; i < 36 * pathNum; i++) {
          int rotDir = int(i / pathNum);
          float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
          if (angDiff > 180.0) {
            angDiff = 360.0 - angDiff;
          }
          if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
              ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle)) {
            continue;
          }
```

对于一个没有被滤除的点（可能不是障碍物，是不平整的起伏路面）score的计算为 1 - （当前的intensity/定值（0.1））

这个点的高度越高（小于被判定为障碍物的最小高度），他的penalty就会越大。而相应的`PenaltyScore`就会就会越小。


```c
          if (clearPathList[i] < pointPerPathThre) {
            //高度对于得分的影响
            float penaltyScore = 1.0 - pathPenaltyList[i] / costHeightThre;
            if (penaltyScore < costScore) penaltyScore = costScore;c
```

$` score = (1 - \sqrt[4]{dirWeight \times dirDiff}) \times rotDirW ^{4} \times penaltyScore `$
这里$rotDirW^{4}$的原因是会有+-90的值，取绝对值。

此处的score越小，则代表cost越小。求最小的score即可。

因此rotDirW所代表的取值倾向为优先选取正前方的方向或者正后方的方向（180-90），（0-90）。

其取值的范围为[0,9]

$dirDiff = fabs(joyDir - endDirPathList[i \% pathNum] - (10.0 * rotDir - 180.0))$

而dirDiff则是每条path在得分中的权重项，得分的高低与path与当前目标点之间的夹角有关，选取夹角尽可能小的路径，会获得更高的score。

其取值的范围为 ：[0,90]单位为弧度

最后值得注意的是每个path的得分最后是加在了该group下
```c
            //每个方向下343条path的不同得分
            float dirDiff = fabs(joyDir - endDirPathList[i % pathNum] - (10.0 * rotDir - 180.0));
            if (dirDiff > 360.0) {
              dirDiff -= 360.0;
            }
            if (dirDiff > 180.0) {
              dirDiff = 360.0 - dirDiff;
            }
            //36个方向的得分（实际遍历的个数是小于36的）
            float rotDirW;
            if (rotDir < 18) rotDirW = fabs(fabs(rotDir - 9) + 1);
            //+1 原因是 rotDir是从0开始的
            //使得rot方向在+-90的范围内
            else rotDirW = fabs(fabs(rotDir - 27) + 1);//同上

            //!score calculation
            float score = (1 - sqrt(sqrt(dirWeight * dirDiff))) * rotDirW * rotDirW * rotDirW * rotDirW * penaltyScore;
            if (score > 0) {
              clearPathPerGroupScore[groupNum * rotDir + pathList[i % pathNum]] += score;
            }
          }
        }
```
选择得分最高的group，该group需要满足：
1. 是当前最高的score
2. 在可视范围之内（前后满足一个即可）[这个也被写死了] 
```c
        float maxScore = 0;
        int selectedGroupID = -1;
        for (int i = 0; i < 36 * groupNum; i++) {
          int rotDir = int(i / groupNum);//仍然是0～35
          float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
          float rotDeg = 10.0 * rotDir;
          if (rotDeg > 180.0) rotDeg -= 360.0;
          if (maxScore < clearPathPerGroupScore[i] && ((rotAng * 180.0 / PI > minObsAngCW && rotAng * 180.0 / PI < minObsAngCCW) || 
              (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive) || !checkRotObstacle)) {
            maxScore = clearPathPerGroupScore[i];
            selectedGroupID = i;
          }
        }
```

如果有选中的group，找出来发布到`\path`的话题中去
```c
        if (selectedGroupID >= 0) {
          int rotDir = int(selectedGroupID / groupNum);
          float rotAng = (10.0 * rotDir - 180.0) * PI / 180;

          selectedGroupID = selectedGroupID % groupNum;
          int selectedPathLength = startPaths[selectedGroupID]->points.size();
          path.poses.resize(selectedPathLength);
          for (int i = 0; i < selectedPathLength; i++) {
            float x = startPaths[selectedGroupID]->points[i].x;
            float y = startPaths[selectedGroupID]->points[i].y;
            float z = startPaths[selectedGroupID]->points[i].z;
            float dis = sqrt(x * x + y * y);

            if (dis <= pathRange / pathScale && dis <= relativeGoalDis / pathScale) {
              path.poses[i].pose.position.x = pathScale * (cos(rotAng) * x - sin(rotAng) * y);
              path.poses[i].pose.position.y = pathScale * (sin(rotAng) * x + cos(rotAng) * y);
              path.poses[i].pose.position.z = pathScale * z;
            } else {
              path.poses.resize(i);
              break;
            }
          }

          path.header.stamp = ros::Time().fromSec(odomTime);
          path.header.frame_id = "vehicle";
          pubPath.publish(path);
        }
```
如果没有找到，则把pathscale减小，继续搜索
```c
        if (selectedGroupID < 0) {
          if (pathScale >= minPathScale + pathScaleStep) {
            pathScale -= pathScaleStep;
            pathRange = adjacentRange * pathScale / defPathScale;
          } else {
            pathRange -= pathRangeStep;
          }
        } else {
          pathFound = true;
          break;
        }
      }
      pathScale = defPathScale;

      if (!pathFound) {
        path.poses.resize(1);
        path.poses[0].pose.position.x = 0;
        path.poses[0].pose.position.y = 0;
        path.poses[0].pose.position.z = 0;

        path.header.stamp = ros::Time().fromSec(odomTime);
        path.header.frame_id = "vehicle";
        pubPath.publish(path);

        #if PLOTPATHSET == 1
        freePaths->clear();
        sensor_msgs::PointCloud2 freePaths2;
        pcl::toROSMsg(*freePaths, freePaths2);
        freePaths2.header.stamp = ros::Time().fromSec(odomTime);
        freePaths2.header.frame_id = "vehicle";
        pubFreePaths.publish(freePaths2);
        #endif
      }

      /*sensor_msgs::PointCloud2 plannerCloud2;
      pcl::toROSMsg(*plannerCloudCrop, plannerCloud2);
      plannerCloud2.header.stamp = ros::Time().fromSec(odomTime);
      plannerCloud2.header.frame_id = "vehicle";
      pubLaserCloud.publish(plannerCloud2);*/
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}

```



