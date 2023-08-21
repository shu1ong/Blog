# [UpdateViewPoints](https://github.com/shu1ong/gitblog/issues/28)

## UpdateViewPoints

关于vierwpoint的生成和选取基本上可以确定其调用过程是通过主函数中`UpdateViewPoints`实现的

这里是对点云文件的一些处理
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
  pd_.viewpoint_manager_->CheckViewPointCollision(pd_.collision_cloud_->cloud_);
  pd_.viewpoint_manager_->CheckViewPointLineOfSight();
  pd_.viewpoint_manager_->CheckViewPointConnectivity();
  ```
然后将对点云文件进行选点的操作
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

对于涉及到的函数进行分别分析,

发现之后的函数主要是对与已经操作的值进行返回（已经对点判断并把相应的值存在了对应的属性中[viewpoint_]）

### ViewPointInCollision

首先是`ViewPointInCollision`，其主要是使用了对于该点的index索引，然后调用相应的功能函数，判断点是否有碰撞？

- [ ] viewpoints_ 装的数据类型因该是每个点的位置表达（绝对位置）positionXYZ

```C++
bool ViewPointManager::ViewPointInCollision(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].InCollision();
}
...
bool InCollision() const
{
  return in_collision_;
}

```

### ViewPointInLineOfSight

- [ ] 比较奇怪的是，这里的并不是一个用作判断的bool函数，仅仅只是return了一个值的调用，这里的数据结构也怪怪的，因为按理来说这里的值不是数组没有储存每个节点viewpoint的信息

这里`in_line_of_sight_`由另一个函数决定`SetInLineOfSight`，但其调用在函数`GetLookAheadPoint`中，那之后再另作分析
```c++
bool ViewPointManager::ViewPointInLineOfSight(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].InLineOfSight();
}

...

bool InLineOfSight() const
  {
    return in_line_of_sight_;
  }
```

### ViewPointConnected

```c++
bool ViewPointManager::ViewPointConnected(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].Connected();
}

.....


```