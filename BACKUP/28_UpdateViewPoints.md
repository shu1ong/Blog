# [UpdateViewPoints](https://github.com/shu1ong/gitblog/issues/28)

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


