# ground_pointcloud_editor
This is a point cloud editor can be used to delete/patch/save point cloud for traversibility map.
```
Rviz is used to edit the point cloud, no extra gui is required!
```

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

## How to play?
```
roslaunch ground_pointcloud_editor test.launch
```

## Documentations
Two buttons with functions are implemented for Rviz. Users can fell free to edit the point cloud in Rviz.

### Button of selected clusters
The selections will be on the clusters, all operations will be based on clusters.
The topic GED_current_selected_clusters shows the selected point cloud.
```
Keyboard d:

Delete selected clusters:
```

The selection from z axis is sliced selection, every time it only selects clusters from max-z to max-z-1.0 which is useful for multiple floors editing (See right gif).
<p float="left">
<img src="https://github.com/tsengapola/my_image_repo/blob/main/ground_editor/clusters_delete.gif" width="400" height="265"/>
<img src="https://github.com/tsengapola/my_image_repo/blob/main/ground_editor/select_slice.gif" width="400" height="265"/>
</p>

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

### Button of selected points
The selection is implemented with aggregated selection.
The topic GED_current_selected_points shows the selected points.

```
Keyboard p:

Patch ground points around selected points:
```
RANSAC is used to find the plane of selected points and fill the hole by iterating the selected area.

<img src="https://github.com/tsengapola/my_image_repo/blob/main/ground_editor/path_around_points.gif" width="400" height="265"/>

```
Keyboard c:

Clear aggregated points selection:
```
<img src="https://github.com/tsengapola/my_image_repo/blob/main/ground_editor/clear_selected_points.gif" width="400" height="265"/>

```
Keyboard d:

Delete selected points:
```
<img src="https://github.com/tsengapola/my_image_repo/blob/main/ground_editor/points_delete.gif" width="400" height="265"/>

