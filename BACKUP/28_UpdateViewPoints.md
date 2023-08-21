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

