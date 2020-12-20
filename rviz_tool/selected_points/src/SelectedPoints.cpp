#include "rviz/selection/selection_manager.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/display_context.h"
#include "rviz/selection/forwards.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/view_manager.h"
#include "rviz/view_controller.h"
#include "OGRE/OgreCamera.h"


#include "selected_points/SelectedPoints.h"

#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <QVariant>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>

#include <pcl/filters/impl/box_clipper3D.hpp>


#include <visualization_msgs/Marker.h>

#include <pcl/filters/crop_box.h>
#include <pcl/common/angles.h>

/**/
#include <std_msgs/String.h>
namespace rviz_plugin_selected_points
{
SelectedPoints::SelectedPoints()
{
    updateTopic();
}

SelectedPoints::~SelectedPoints()
{
}

void SelectedPoints::updateTopic()
{
    nh_.param("frame_id", tf_frame_, std::string("/base_link"));
    rviz_cloud_topic_ = std::string("/rviz_selected_points");
    bb_marker_topic_ = std::string("/rviz_selected_bounding_box");
    keyboard_topic_ = std::string("/rviz_selected_points_key");

    rviz_selected_pub_ = nh_.advertise<sensor_msgs::PointCloud2>( rviz_cloud_topic_.c_str(), 1 );
    //bb_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(bb_marker_topic_.c_str(), 1);
    key_pub_ = nh_.advertise<std_msgs::String>( keyboard_topic_.c_str(), 2 );
    
    ROS_INFO_STREAM_NAMED("SelectedPoints.updateTopic", "Publishing rviz selected points on topic " <<  nh_.resolveName (rviz_cloud_topic_) );//<< " with frame_id " << context_->getFixedFrame().toStdString() );

    accumulated_selected_pc_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    num_acc_points_ = 0;
    num_selected_points_ = 0;
}

int SelectedPoints::processKeyEvent( QKeyEvent* event, rviz::RenderPanel* panel )
{
    if(event->type() == QKeyEvent::KeyPress)
    {
        std_msgs::String key_press;
        key_press.data += event->key();
        key_pub_.publish(key_press);

        if(event->key() == 'c' || event->key() == 'C')
        {
            this->_clearAccumulatedPoints();
        }
        else if(event->key() == 'd' || event->key() == 'D')
        {
            /*Delete points, so reset selection*/
            accumulated_selected_pc_.reset(new pcl::PointCloud<pcl::PointXYZ>);
            this->num_acc_points_ = 0;
        }
        else if(event->key() == 'p' || event->key() == 'P')
        {
            /*Delete points, so reset selection*/
            accumulated_selected_pc_.reset(new pcl::PointCloud<pcl::PointXYZ>);
            this->num_acc_points_ = 0;
        }
    }
}

int SelectedPoints::processMouseEvent( rviz::ViewportMouseEvent& event )
{
    int flags = rviz::SelectionTool::processMouseEvent( event );

    // determine current selection mode
    if( event.alt() )
    {
        selecting_ = false;
    }
    else
    {
        if( event.leftDown() )
        {
            selecting_ = true;
        }
    }

    if( selecting_ )
    {
        if( event.leftUp() )
        {
            ROS_INFO_STREAM_NAMED( "SelectedPoints.processKeyEvent", "Using selected area to find a new bounding box and publish the points inside of it");
            this->_processSelectedAreaAndFindPoints();
        }
    }
    return flags;
}

int SelectedPoints::_processSelectedAreaAndFindPoints()
{
    rviz::SelectionManager* sel_manager = context_->getSelectionManager();
    rviz::M_Picked selection = sel_manager->getSelection();
    rviz::PropertyTreeModel *model = sel_manager->getPropertyModel();
    int num_points = model->rowCount();
    ROS_INFO_STREAM_NAMED( "SelectedPoints._processSelectedAreaAndFindPoints", "Number of points in the selected area: " << num_points);

    // Generate a ros point cloud message with the selected points in rviz
    sensor_msgs::PointCloud2 selected_points_ros;
    selected_points_ros.header.frame_id = context_->getFixedFrame().toStdString();
    selected_points_ros.height = 1;
    selected_points_ros.width = num_points;
    selected_points_ros.point_step = 3 * 4;
    selected_points_ros.row_step = num_points * selected_points_ros.point_step;
    selected_points_ros.is_dense = false;
    selected_points_ros.is_bigendian = false;

    selected_points_ros.data.resize( selected_points_ros.row_step );
    selected_points_ros.fields.resize( 3 );

    selected_points_ros.fields[0].name = "x";
    selected_points_ros.fields[0].offset = 0;
    selected_points_ros.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    selected_points_ros.fields[0].count = 1;

    selected_points_ros.fields[1].name = "y";
    selected_points_ros.fields[1].offset = 4;
    selected_points_ros.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    selected_points_ros.fields[1].count = 1;

    selected_points_ros.fields[2].name = "z";
    selected_points_ros.fields[2].offset = 8;
    selected_points_ros.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    selected_points_ros.fields[2].count = 1;

    for( int i = 0; i < num_points; i++ )
    {
        QModelIndex child_index = model->index( i, 0 );
        rviz::Property* child = model->getProp( child_index );
        rviz::VectorProperty* subchild = (rviz::VectorProperty*) child->childAt( 0 );
        Ogre::Vector3 vec = subchild->getVector();

        uint8_t* ptr = &selected_points_ros.data[0] + i * selected_points_ros.point_step;
        *(float*)ptr = vec.x;
        ptr += 4;
        *(float*)ptr = vec.y;
        ptr += 4;
        *(float*)ptr = vec.z;
        ptr += 4;
    }
    selected_points_ros.header.stamp = ros::Time::now();

    // Convert the ros point cloud message with the selected points into a pcl point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr selected_points_pcl(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(selected_points_ros, *selected_points_pcl);
    *accumulated_selected_pc_ += *selected_points_pcl;

    sensor_msgs::PointCloud2 accumulated_selected_ros;
    pcl::toROSMsg(*this->accumulated_selected_pc_, accumulated_selected_ros);
    accumulated_selected_ros.header.frame_id = context_->getFixedFrame().toStdString();
    rviz_selected_pub_.publish(accumulated_selected_ros);

    /**/
    // Generate an oriented bounding box around the selected points in RVIZ
    // Compute principal direction
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*accumulated_selected_pc_, centroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*accumulated_selected_pc_, centroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
    eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

    // Move the points to the that reference frame
    Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
    p2w.block<3,3>(0,0) = eigDx.transpose();
    p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
    pcl::PointCloud<pcl::PointXYZ> cPoints;
    pcl::transformPointCloud(*accumulated_selected_pc_, cPoints, p2w);

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(cPoints, min_pt, max_pt);
    const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());
    // Final transform and bounding box size
    const Eigen::Quaternionf qfinal(eigDx);
    const Eigen::Vector3f tfinal = eigDx*mean_diag + centroid.head<3>();
    double bb_size_x = max_pt.x - min_pt.x;
    double bb_size_y = max_pt.y - min_pt.y;
    double bb_size_z = max_pt.z - min_pt.z;

    // Publish the bounding box as a rectangular marker
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = context_->getFixedFrame().toStdString().c_str();
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = tfinal.x();
    marker.pose.position.y = tfinal.y();
    marker.pose.position.z = tfinal.z();
    marker.pose.orientation.x = qfinal.x();
    marker.pose.orientation.y = qfinal.y();
    marker.pose.orientation.z = qfinal.z();
    marker.pose.orientation.w = qfinal.w();
    marker.scale.x = bb_size_x;
    marker.scale.y = bb_size_y;
    marker.scale.z = bb_size_z;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5;
    marker.lifetime = ros::Duration();
    //bb_marker_pub_.publish(marker);

    return 0;
}

int SelectedPoints::_clearAccumulatedPoints()
{

    accumulated_selected_pc_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->num_acc_points_ = 0;

    /*Clear rviz*/
    sensor_msgs::PointCloud2 accumulated_selected_ros;
    pcl::toROSMsg(*this->accumulated_selected_pc_, accumulated_selected_ros);
    accumulated_selected_ros.header.frame_id = context_->getFixedFrame().toStdString();
    rviz_selected_pub_.publish(accumulated_selected_ros);

    return 0;
}

} // end namespace rviz_plugin_selected_points_topic

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz_plugin_selected_points::SelectedPoints, rviz::Tool )
