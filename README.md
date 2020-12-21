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
[![alt text](https://github.com/tsengapola/my_image_repo/blob/main/ground_editor/demo_main.gif)](https://www.youtube.com/watch?v=5-HCcPmUQBg "Ground point cloud editor")



Since this is a editor, the 'last step' function is provided.