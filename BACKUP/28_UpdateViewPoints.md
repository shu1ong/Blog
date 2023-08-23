# [UpdateViewPoints](https://github.com/shu1ong/gitblog/issues/28)

- [UpdateViewPoints](#updateviewpoints)
  - [`CheckViewPointCollision()`](#checkviewpointcollision)
    - [`CheckViewPointCollisionWithCollisionGrid()`](#checkviewpointcollisionwithcollisiongrid)
    - [`heckViewPointBoundaryCollision()`](#heckviewpointboundarycollision)
  - [`CheckViewPointLineOfSight()`](#checkviewpointlineofsight)
  - [`CheckViewPointConnectivity()`](#checkviewpointconnectivity)
- [GetViewPointCandidate](#getviewpointcandidate)
- [其中比较重要的就是这条判断的条件语句：](#其中比较重要的就是这条判断的条件语句)


## UpdateViewPoints

关于vierwpoint的生成和选取基本上可以确定其调用过程是通过主函数中`UpdateViewPoints`实现的

这里是对点云文件进行一个拼接
```C++
int SensorCoveragePlanner3D::UpdateViewPoints()
{
  ...
  
  if (pp_.kUseTerrainHeight)
  {
    pd_.viewpoint_manager_->SetViewPointHeightWithTerrain(pd_.large_terrain_cloud_->cloud_);
  }
  if (pp_.kCheckTerrainCollision)//true
  {
    *(pd_.collision_cloud_->cloud_) += *(pd_.terrain_collision_cloud_->cloud_);
    *(pd_.collision_cloud_->cloud_) += *(pd_.terrain_ext_collision_cloud_->cloud_);
  }
  ```
然后把拼接过后的点云文件进行筛选，一共使用了三个函数对其进行筛选
  ```c++
  pd_.viewpoint_manager_->CheckViewPointCollision(pd_.collision_cloud_->cloud_);
  pd_.viewpoint_manager_->CheckViewPointLineOfSight();
  pd_.viewpoint_manager_->CheckViewPointConnectivity();
  ```
三个对应的函数分别是

[CheckViewPointCollision()](#CheckViewPointCollision)

[`CheckViewPointLineOfSight()`](#CheckViewPointLineOfSight) 

[`CheckViewPointConnectivity()`](#CheckViewPointConnectivity)


对于涉及到的函数进行分别分析,

### `CheckViewPointCollision()`

下面前嵌套了俩个函数，而对于输入的参数`pd_.collision_cloud_`是对terrain map的叠加
```c++
void ViewPointManager::CheckViewPointCollision(const pcl::PointCloud<pcl::PointXYZI>::Ptr& collision_cloud)
{
  CheckViewPointCollisionWithCollisionGrid(collision_cloud);
  CheckViewPointBoundaryCollision();
}
```
#### `CheckViewPointCollisionWithCollisionGrid()`
```c++
void ViewPointManager::CheckViewPointCollisionWithCollisionGrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr& collision_cloud) {
  //将collion属性全部初始化为true,因为点云本来就是collision cloud
  for (int i = 0; i < viewpoints_.size(); i++) {
    if (ViewPointInCollision(i, true)) {
      AddViewPointCollisionFrameCount(i, true);
    }
  }
  //设置cell的原点
  std::fill(collision_point_count_.begin(), collision_point_count_.end(), 0);
  collision_grid_origin_ = origin_ - Eigen::Vector3d::Ones() * vp_.kViewPointCollisionMargin; // ???设置原点
  collision_grid_->SetOrigin(collision_grid_origin_);
  //遍历点云中的每一个点
  for (const auto& point : collision_cloud->points)
  {
    //首先获取其xyz坐标，转化为下标式的格式（从离散点转化为grid空间中的连续表达）
    Eigen::Vector3i collision_grid_sub = collision_grid_->Pos2Sub(point.x, point.y, point.z);//这里是对所有的点进行操作？
    //return ind >= 0 && ind < cell_number_; InRange指的是在local terrain range
    //grid坐标以分辨率和grid数量进行构建
    if (collision_grid_->InRange(collision_grid_sub))
    {
      //将gird坐标再转化成按序列排布的ind
      int collision_grid_ind = collision_grid_->Sub2Ind(collision_grid_sub);
      collision_point_count_[collision_grid_ind]++;
      //当一个grid中点的个数超过一定限制后
      if (collision_point_count_[collision_grid_ind] >= vp_.kCollisionPointThr)
      {
        //返回该cell的序列
        std::vector<int> collision_viewpoint_indices = collision_grid_->GetCellValue(collision_grid_ind);
        //这里虽然getCellValue返回来只有一个值，但是必须和cell的类型进行匹配，所以给了一个Vector的向量，因此当需要提取这个值的时候必须使用遍历
        for (int i = 0; i < collision_viewpoint_indices.size(); i++)
        {
          int viewpoint_ind = collision_viewpoint_indices[i];
          MY_ASSERT(viewpoint_ind >= 0 && viewpoint_ind < vp_.kViewPointNumber);
          double z_diff = point.z - GetViewPointHeight(viewpoint_ind);
          if ((z_diff >= 0 && z_diff <= vp_.kViewPointCollisionMarginZPlus) ||
              (z_diff < 0 && z_diff >= -vp_.kViewPointCollisionMarginZMinus))
          {
            //将与点云干涉的点进行标记
            SetViewPointCollision(viewpoint_ind, true);
            //那些被抠掉的节点的FrameCount参数置false（0）
            ResetViewPointCollisionFrameCount(viewpoint_ind);
          }
        }
      }
    }
  }
}
```
#### `heckViewPointBoundaryCollision()`
```c++
void ViewPointManager::CheckViewPointBoundaryCollision()
{
  // Check for the polygon boundary and nogo zones
  for (int i = 0; i < vp_.kViewPointNumber; i++)
  {
    geometry_msgs::Point viewpoint_position = GetViewPointPosition(i, true);
    if ((!viewpoint_boundary_.points.empty() &&
         !misc_utils_ns::PointInPolygon(viewpoint_position, viewpoint_boundary_)))
    {
      SetViewPointCollision(i, true, true);
      continue;
    }
    for (int j = 0; j < nogo_boundary_.size(); j++)
    {
      if (!nogo_boundary_[j].points.empty() && misc_utils_ns::PointInPolygon(viewpoint_position, nogo_boundary_[j]))
      {
        SetViewPointCollision(i, true, true);

        break;
      }
    }
  }
}
```

### `CheckViewPointLineOfSight()`

整段代码主要的工作是检测Viewpoint有无超出划定的边界，尤其是在plugon boudary进行更新的时候。

分别检测了三种情况，即local区域的6个表面。

- [ ] 比较confusing的是checked的Vector不知道其作用
```c++
void ViewPointManager::CheckViewPointLineOfSight()
{
  if (!initialized_)
    return;

  for (int i = 0; i < viewpoints_.size(); i++)
  {
    SetViewPointInCurrentFrameLineOfSight(i, false, true); //set the value to false
  }

  Eigen::Vector3i robot_sub = GetViewPointSub(robot_position_);
  MY_ASSERT(grid_->InRange(robot_sub));
  int robot_viewpoint_ind = grid_->Sub2Ind(robot_sub);
  
  //init the property
  //false means the ind has not get need to use function to claculate
  // 其默认值为false void SetViewPointInLineOfSight(int viewpoint_ind, bool in_line_of_sight, bool use_array_ind = false);
  
  SetViewPointInLineOfSight(robot_viewpoint_ind, true);
  SetViewPointInCurrentFrameLineOfSight(robot_viewpoint_ind, true);

  std::vector<bool> checked(vp_.kViewPointNumber, false);
  std::vector<Eigen::Vector3i> ray_cast_cells;
  Eigen::Vector3i max_sub(vp_.kNumber.x() - 1, vp_.kNumber.y() - 1, vp_.kNumber.z() - 1);
  Eigen::Vector3i min_sub(0, 0, 0);

  int x_indices[2] = { 0, vp_.kNumber.x() - 1 };//总共只有两个数
  int y_indices[2] = { 0, vp_.kNumber.y() - 1 };
  int z_indices[2] = { 0, vp_.kNumber.z() - 1 };
  
  //x = o 和 x = end 两个端面上所有的点
  for (int xi = 0; xi < 2; xi++)
  {
    for (int y = 0; y < vp_.kNumber.y(); y++)
    {
      for (int z = 0; z < vp_.kNumber.z(); z++)
      {
        int x = x_indices[xi];
        Eigen::Vector3i end_sub(x, y, z);//端点
        int array_ind = grid_->GetArrayInd(end_sub);
        if (!checked[array_ind])
        {
          CheckViewPointLineOfSightHelper(robot_sub, end_sub, max_sub, min_sub);
          checked[array_ind] = true;
        }
      }
    }
  }

  //y = 0 和 y = y.end 两个端面上所有的点
  for (int x = 0; x < vp_.kNumber.x(); x++)
  {
    for (int yi = 0; yi < 2; yi++)
    {
      for (int z = 0; z < vp_.kNumber.z(); z++)
      {
        int y = y_indices[yi];
        Eigen::Vector3i end_sub(x, y, z);
        int array_ind = grid_->GetArrayInd(end_sub);
        if (!checked[array_ind])
        {
          CheckViewPointLineOfSightHelper(robot_sub, end_sub, max_sub, min_sub);
          checked[array_ind] = true;
        }
      }
    }
  }

  //z = 0 and z = z.end上所有的点
  for (int x = 0; x < vp_.kNumber.x(); x++)
  {
    for (int y = 0; y < vp_.kNumber.y(); y++)
    {
      for (int zi = 0; zi < 2; zi++)
      {
        int z = z_indices[zi];
        Eigen::Vector3i end_sub(x, y, z);
        int array_ind = grid_->GetArrayInd(end_sub);
        if (!checked[array_ind])
        {
          CheckViewPointLineOfSightHelper(robot_sub, end_sub, max_sub, min_sub);
          checked[array_ind] = true;
        }
      }
    }
  }
}

```
- [ ] 其中重要的判断函数为`CheckViewPointLineOfSightHelper`

### `CheckViewPointConnectivity()`

检查Viewpoint是否是可到达的，但原理来讲相对粗糙：邻近的9点都不能有碰撞干涉
```c++
void ViewPointManager::CheckViewPointConnectivity()
{
  if (!initialized_) return;


  Eigen::Vector3i robot_sub = GetViewPointSub(robot_position_);
  MY_ASSERT(grid_->InRange(robot_sub));
  int robot_ind = grid_->Sub2Ind(robot_sub);
  int robot_array_ind = grid_->GetArrayInd(robot_sub);

  //先检查及机器人的位置是否与碰撞干涉
  if (ViewPointInCollision(robot_ind))
  {
    // std::cout << "ViewPointManager::CheckViewPointConnectivity: robot in collision" << std::endl;
    // return;
    // Find the nearest viewpoint that is not in collision
    bool found_collision_free_viewpoint = false;
    double min_dist_to_robot = DBL_MAX;

    //遍历所有ViewPoint（涉及到的ind的操作）
    for (int i = 0; i < vp_.kViewPointNumber; i++)
    {
      int array_ind = grid_->GetArrayInd(i);
      //是否与障碍物干涉
      if (!ViewPointInCollision(i))
      {
        //获取个点的位置
        geometry_msgs::Point position = GetViewPointPosition(i);
        Eigen::Vector3d viewpoint_position(position.x, position.y, position.z);
        //计算每个点到机器人的距离
        double dist_to_robot = (viewpoint_position - robot_position_).norm();
        //找到其中离机器人距离最近的Viewpoint
        if (dist_to_robot < min_dist_to_robot)
        { 
          
          min_dist_to_robot = dist_to_robot;
          robot_ind = i;
          robot_array_ind = array_ind;
          found_collision_free_viewpoint = true;
        }
      }
    }
    //没找到则直接退出
    if (!found_collision_free_viewpoint)
    {
      std::cout << "All viewpoints in collision, exisiting" << std::endl;
      return;
    }
  }

  // inti the connected property to false
  for (auto& viewpoint : viewpoints_)
  {
    viewpoint.SetConnected(false);
  }
  std::vector<bool> checked(vp_.kViewPointNumber, false);
  checked[robot_ind] = true;
  SetViewPointConnected(robot_ind, true);
  std::list<int> queue;
  queue.push_back(robot_ind);
  int connected_viewpoint_count = 1;

  //只要能够找到离robo最近的Viewpoint，且不在障碍物的干涉中
  while (!queue.empty())
  {
    int cur_ind = queue.front();
    queue.pop_front();
    //遍历邻近的9个grid，检查是否都在polygon range内
    for (int i = 0; i < connected_neighbor_indices_[cur_ind].size(); i++)
    {
      int neighbor_ind = connected_neighbor_indices_[cur_ind][i];
      //如果不在
      if (!grid_->InRange(neighbor_ind))
      {
        std::cout << "ViewPointManager::CheckViewPointConnectivity: neighbor ind out of bound" << std::endl;
        continue;
      }

      //如果满足要求
      if (!checked[neighbor_ind] && !ViewPointInCollision(neighbor_ind) && ViewPointInLineOfSight(neighbor_ind))
      {
        //当前的这个ViewPoint的高度和周围的高度应该小于一定的阈值，这个差值的大小很大程度上取决于你自己的分表率reselution的设计
        if (std::abs(GetViewPointHeight(cur_ind) - GetViewPointHeight(neighbor_ind)) < vp_.kConnectivityHeightDiffThr)
        {
          SetViewPointConnected(neighbor_ind, true);
          connected_viewpoint_count++;
          queue.push_back(neighbor_ind);
        }
      }
      checked[neighbor_ind] = true;
    }
  }
}
```






---

```c++
  //!get viewpoint candidate
  int viewpoint_candidate_count = pd_.viewpoint_manager_->GetViewPointCandidate();
```

然后对viewpoint进行队列更新，将cllosion_cloud发布出去
```c++
  UpdateVisitedPositions();
  pd_.viewpoint_manager_->UpdateViewPointVisited(pd_.visited_positions_);
  pd_.viewpoint_manager_->UpdateViewPointVisited(pd_.grid_world_);

  // For visualization
  pd_.collision_cloud_->Publish();
  // pd_.collision_grid_cloud_->Publish();
  pd_.viewpoint_manager_->GetCollisionViewPointVisCloud(pd_.viewpoint_in_collision_cloud_->cloud_);
  pd_.viewpoint_in_collision_cloud_->Publish();

  viewpoint_manager_update_timer.Stop(false);
  return viewpoint_candidate_count;
}
```


---
## GetViewPointCandidate
接下来分析选点的函数`GetViewPointCandidate`


```C++
int ViewPointManager::GetViewPointCandidate()
{
  viewpoint_candidate_cloud_->clear();
  viewpoint_in_collision_cloud_->clear();
  candidate_indices_.clear();
  for (int i = 0; i < vp_.kViewPointNumber; i++)//viewpoint的总数是由配置文件决定的，在每边上决定布种的个数，说明在区域中的布种是均布的
  {
    SetViewPointCandidate(i, false);//init 将所有的点初始化为false
    if (!ViewPointInCollision(i) && ViewPointInLineOfSight(i) && ViewPointConnected(i))//!the condition need to be satified
    { 
      //改变当前被选点的状态为true
      //将被选点的索引返回
      //求取当前被选点的位置并返回
      SetViewPointCandidate(i, true);
      candidate_indices_.push_back(i);
      geometry_msgs::Point viewpoint_position = GetViewPointPosition(i);
      pcl::PointXYZI point;
      point.x = viewpoint_position.x;
      point.y = viewpoint_position.y;
      point.z = viewpoint_position.z;
      viewpoint_candidate_cloud_->points.push_back(point);
    }
    //被遮挡的节点
    if (ViewPointInCollision(i))
    {
      geometry_msgs::Point viewpoint_position = GetViewPointPosition(i);
      pcl::PointXYZI point;
      point.x = viewpoint_position.x;
      point.y = viewpoint_position.y;
      point.z = viewpoint_position.z;
      point.intensity = GetViewPointCollisionFrameCount(i);
      viewpoint_in_collision_cloud_->points.push_back(point);
    }
  }
  // std::cout << "candidate viewpoint num: " << candidate_indices_.size() << std::endl;
  //如果没有找到备选节点
  if (!candidate_indices_.empty())
  {
    kdtree_viewpoint_candidate_->setInputCloud(viewpoint_candidate_cloud_);//todo？？？？
  }

  if (!viewpoint_in_collision_cloud_->points.empty())
  {
    kdtree_viewpoint_in_collision_->setInputCloud(viewpoint_in_collision_cloud_);
  }

  // Construct a graph of all the viewpoints
  GetCandidateViewPointGraph(candidate_viewpoint_graph_, candidate_viewpoint_dist_, candidate_viewpoint_position_);

  return candidate_indices_.size();//返回选择节点的个数
}

```


## 其中比较重要的就是这条判断的条件语句：
```c++
!ViewPointInCollision(i) && ViewPointInLineOfSight(i) && ViewPointConnected(i)
```
发现之后的函数主要是对与已经操作的值进行返回（已经对点判断并把相应的值存在了对应的属性中[viewpoint_]）
