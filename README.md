# ground_pointcloud_editor
This is a point cloud editor can be used to delete/patch/save point cloud for traversibility map.

This application comprises two parts:
1. Customized rviz tools which procide the editing functions.
2. The node which comprises edited data structures and algorithm.
  a. The region growing algorithm is first used to segment the whole point cloud.
  b. The first tool is used to edit the point cloud by selecting clusters.
  c. The second tool is used to edit the point cloud by selecting points.

## Required package
[pcl_ros](http://wiki.ros.org/pcl_ros)

## Demo: edit point cloud by selecting clusters and points

[See full video in youtube](https://www.youtube.com/watch?v=5-HCcPmUQBg)

[![alt text](https://github.com/tsengapola/my_image_repo/blob/main/ground_editor/demo_main.gif)](https://www.youtube.com/watch?v=5-HCcPmUQBg "Ground point cloud editor")


## Documentations

### Using selected clusters
The selections will be on the clusters, all operations will be based on clusters.
The topic GED_current_selected_clusters shows the selected point cloud.
```
Keyboard d:

Delete selected clusters: 
```
<img src="https://github.com/tsengapola/my_image_repo/blob/main/ground_editor/clusters_delete.gif" width="400" height="265"/>
```
Keyboard w:

Delete wall clusters: 
```
<img src="https://github.com/tsengapola/my_image_repo/blob/main/ground_editor/wall_delete.gif" width="400" height="265"/>
```
Keyboard r:

Reset all clusters: 
```
<img src="https://github.com/tsengapola/my_image_repo/blob/main/ground_editor/reset_all.gif" width="400" height="265"/>
```
Keyboard k:

Save result point cloud in /tmp/ground.pcd with pcl::PointXYZ format
```
```
Keyboard z:

Return last step
```
<img src="https://github.com/tsengapola/my_image_repo/blob/main/ground_editor/last_step.gif" width="400" height="265"/>
