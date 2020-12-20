////////////////////////////////////////////////////////////////////////////////
//
// Filename:      selected_points_topic.h
// Last change:   2013-11-21
// Authors:       Apola
// Documentation: http://docs.ros.org/api/rviz/html/
// Version:       1.0.0
//
//////////////////////////////// DOCUMENTATION /////////////////////////////////
//
// Fork of the rviz::SelectionTool:
// Drag with the left button to select objects in the 3D scene.
// Hold the Alt key to change viewpoint as in the Move tool.
// Additionally publishes selected points on /selected_points topic.
//
/////////////////////////////////// LICENSE ////////////////////////////////////
//
// Copyright (C) 2013 Robotics & Biology Laboratory (RBO) TU Berlin
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////// CHANGELOG ///////////////////////////////////
//
// Version 1.0.0 (2013-11-21)
//
//////////////////////////////////// NOTES /////////////////////////////////////
//
// TODO:
//
////////////////////////////////////////////////////////////////////////////////

#ifndef SELECTED_CLUSTERS_H
#define SELECTED_CLUSTERS_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <ros/node_handle.h>
# include <ros/publisher.h>

# include "rviz/tool.h"

# include <QCursor>
# include <QObject>
#endif

#include "rviz/default_plugin/tools/selection_tool.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/extract_indices.h>

namespace rviz_plugin_selected_clusters
{

class SelectedClusters;

class SelectedClusters : public rviz::SelectionTool
{
Q_OBJECT
public:
  SelectedClusters();
  virtual ~SelectedClusters();

  /*
   * Hooks on rviz::SelectionTool::processMouseEvent() to get and publish
   * selected points
   */
  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );

  virtual int processKeyEvent( QKeyEvent* event, rviz::RenderPanel* panel );

public Q_SLOTS:
  /*
   * Creates the ROS topic
   */
  void updateTopic();

  void PointCloudsCallback(const sensor_msgs::PointCloud2ConstPtr &pc_msg);

protected:

  int _processSelectedAreaAndFindPoints();
  int _publishAccumulatedPoints();
  ros::NodeHandle nh_;
  ros::Publisher rviz_selected_pub_;
  ros::Publisher real_selected_pub_;
  ros::Publisher partial_pc_pub_;
  ros::Publisher bb_marker_pub_;
  ros::Publisher key_pub_;
  ros::Subscriber pc_subs_;

  std::string tf_frame_;
  std::string rviz_cloud_topic_;
  std::string real_cloud_topic_;
  std::string subs_cloud_topic_;
  std::string bb_marker_topic_;
  std::string keyboard_topic_;
  bool selecting_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_pc_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr selected_segment_pc_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_segment_pc_;

  pcl::ExtractIndices<pcl::PointXYZRGB>::Ptr extract_indices_filter_;

  int num_acc_points_;
  int num_selected_points_;
};
} // end namespace rviz_plugin_selected_clusters

#endif // SELECTED_CLUSTERS_H

////////////////////////////////////////////////////////////////////////////////

