# ground_pointcloud_editor
This is a point cloud editor can be used to delete/patch/save point cloud for traversibility map.

> Rviz is used to edit the point cloud, no extra gui is required!

> This tool can patch the holes on the ground using RANSAC to find the optimal plane!

<img src="https://github.com/tsengapola/my_image_repo/blob/main/ground_editor/path_around_points.gif" width="400" height="265"/>

This application comprises two parts:
1. Customized rviz tools for the editing functions.
2. The ros node that read pcd file and provides editing functions with algorithms (for example, segmentation for wall removal.).

  * The region growing algorithm is used to segment the whole point cloud when .pcd is imported.
  
  * The first tool in Rviz is used to edit the point cloud by selecting clusters.
  
  * The second tool in Rviz is used to edit the point cloud by selecting points.

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
Two tools with functions are implemented for Rviz. Users just need to use these two tools to edit the point cloud in Rviz.

### Button of selected clusters
The selections will be on the clusters, all operations will be affected on clusters.
The topic GED_current_selected_clusters shows the selected point cloud.
```
Keyboard d:

Delete selected clusters:
```

The selection from z axis is sliced selection, it only selects clusters from max_z to max_z-1.0 which is useful for multiple floors editing (See right .gif).
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

Return to last step
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

