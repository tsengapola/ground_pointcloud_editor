#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

/*For pcl::fromROSMsg*/
#include <pcl_ros/point_cloud.h>
/*For pcl::transformPointCloud*/
#include <pcl_ros/transforms.h>

/*For ground detection tutorial*/
#include <vector>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

/*keyboard*/
#include <std_msgs/String.h>

/*write point cloud to pcd*/
#include <pcl/io/pcd_io.h>

/*box detection*/
#include <pcl/common/common.h>
#include <pcl/filters/impl/box_clipper3D.hpp>
#include <visualization_msgs/Marker.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/angles.h>

/*RANSAC*/
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

typedef std::pair<std::vector <pcl::PointIndices>, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> operation_pair;

class Ground_PointCloud_Editor{
    public:
        Ground_PointCloud_Editor(ros::NodeHandle& nh, ros::NodeHandle& pnh);
        ~Ground_PointCloud_Editor();
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        std::string global_frame_;

        /*Pub*/
        ros::Publisher pub_raw_pc_;
        ros::Publisher pub_raw_transformed_pc_;
        ros::Publisher pub_op_current_pc_;
        ros::Publisher pub_op_current_remaining_pc_;
        ros::Publisher pub_op_current_aggregated_pc_;
        ros::Publisher pub_bb_marker_;

        /*Sub*/
        ros::Subscriber pcl_ros_sub_;
        ros::Subscriber pcl_rviz_selected_clusters_sub_;
        ros::Subscriber pcl_rviz_selected_points_sub_;
        ros::Subscriber rviz_clusters_keyboard_sub_;
        ros::Subscriber rviz_points_keyboard_sub_;

        pcl::PointCloud<pcl::PointXYZI>::Ptr pc_original_z_up_;
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> patch_pcs_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc_aggregated_;
        pcl::PointCloud<pcl::Normal>::Ptr normals_;
        std::vector <pcl::PointIndices> clusters_;
        std::vector<operation_pair> clusters_op_steps_;

        /*kdtree for rviz selection*/
        std::map< std::vector<pcl::PointIndices>::iterator ,pcl::KdTreeFLANN<pcl::PointXYZI> > map_kdtree_current_remaining_; 
        std::set<std::vector<pcl::PointIndices>::iterator> selected_clusters_;

        double rotate_around_x_, rotate_around_y_, rotate_around_z_;
        double translate_x_, translate_y_, translate_z_;

        void cbMapcloud(const sensor_msgs::PointCloud2::ConstPtr& msg);
        void cbKeyBoardClusters(const std_msgs::String::ConstPtr& msg);
        void cbKeyBoardPoints(const std_msgs::String::ConstPtr& msg);
        void cbRvizSelectedClusters(const sensor_msgs::PointCloud2::ConstPtr& msg);
        void cbRvizSelectedPoints(const sensor_msgs::PointCloud2::ConstPtr& msg);

        void funcLastStep();
        void funcResetClusters();
        void funcRemoveWalls();
        void funcDeleteSelectedClusters();
        void funcDeleteAggregatedPoints();
        void funcSavePC();
        void funcPatchPlanar();
        void funcQueueOperation();

        void updateSelectedCluster(); //this function should be called whenever a selection is triggered
        void pubCurrentRemainingPC(); //this function should be called whenever a cluster is deleted
};

Ground_PointCloud_Editor::Ground_PointCloud_Editor(ros::NodeHandle& nh, ros::NodeHandle& pnh){
    nh_ = nh;
    pnh_ = pnh;

    pnh_.param("rotate_around_x", rotate_around_x_, 0.0);
    pnh_.param("rotate_around_y", rotate_around_y_, 0.0);
    pnh_.param("rotate_around_z", rotate_around_z_, 0.0);
    pnh_.param("translate_x", translate_x_, 0.0);
    pnh_.param("translate_y", translate_y_, 0.0);
    pnh_.param("translate_z", translate_z_, 0.0);

    global_frame_ = std::string("map");

    pc_aggregated_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

    pcl_rviz_selected_clusters_sub_ = nh_.subscribe("rviz_selected_clusters", 2, &Ground_PointCloud_Editor::cbRvizSelectedClusters, this);
    pcl_rviz_selected_points_sub_ = nh_.subscribe("rviz_selected_points", 2, &Ground_PointCloud_Editor::cbRvizSelectedPoints, this);
    pcl_ros_sub_ = nh_.subscribe("mapcloud", 2, &Ground_PointCloud_Editor::cbMapcloud, this);
    rviz_clusters_keyboard_sub_ = nh_.subscribe("rviz_selected_clusters_key", 2, &Ground_PointCloud_Editor::cbKeyBoardClusters, this);
    rviz_points_keyboard_sub_ = nh_.subscribe("rviz_selected_points_key", 2, &Ground_PointCloud_Editor::cbKeyBoardPoints, this);

    pub_raw_pc_ = pnh_.advertise<sensor_msgs::PointCloud2>("GED_raw_pc", 2, true);
    pub_raw_transformed_pc_ = pnh_.advertise<sensor_msgs::PointCloud2>("GED_raw_transformed_pc", 2, true);
    pub_op_current_remaining_pc_ = pnh_.advertise<sensor_msgs::PointCloud2>("GED_op_current_remaining_pc", 2, true);
    pub_op_current_pc_ = pnh_.advertise<sensor_msgs::PointCloud2>("GED_current_selected_clusters", 2, true);
    pub_op_current_aggregated_pc_ = pnh_.advertise<sensor_msgs::PointCloud2>("GED_current_selected_points", 2, true);
    pub_bb_marker_ = nh_.advertise<visualization_msgs::Marker>("GED_current_aggregated_bbox", 1);

}

Ground_PointCloud_Editor::~Ground_PointCloud_Editor(){

}

void Ground_PointCloud_Editor::funcQueueOperation(){
  operation_pair a_op_pair;
  a_op_pair.first = clusters_;
  a_op_pair.second = patch_pcs_;
  clusters_op_steps_.push_back(a_op_pair);
}

void Ground_PointCloud_Editor::updateSelectedCluster(){

  /*New point cloud for publish*/ 
  pcl::PointCloud<pcl::PointXYZI>::Ptr selected_clusters_pc(new pcl::PointCloud<pcl::PointXYZI>); 
  for(auto select_it = selected_clusters_.begin(); select_it!=selected_clusters_.end(); select_it++){
    
    auto it = (*(*select_it));
    
    for (std::vector<int>::const_iterator pit = it.indices.begin (); pit != it.indices.end (); ++pit){
      selected_clusters_pc->points.push_back (pc_original_z_up_->points[*pit]);
    }  
    
  }

  pcl_conversions::toPCL(ros::Time::now(), selected_clusters_pc->header.stamp);
  selected_clusters_pc->header.frame_id = global_frame_;
  pub_op_current_pc_.publish(selected_clusters_pc);    

}

void Ground_PointCloud_Editor::pubCurrentRemainingPC(){

  /*The remaining point cloud is changed, reset the map_kdtree for rviz selection to increase speed*/
  map_kdtree_current_remaining_.clear();
  
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr current_remaining_pc (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr current_remaining_pc_plus_patch (new pcl::PointCloud<pcl::PointXYZI>);

  int intensity_cnt = 0;

  for (std::vector<pcl::PointIndices>::iterator it = clusters_.begin (); it != clusters_.end (); ++it)
  {

    if(it->indices.empty()) //We remove all points in this cluster
      continue;
    /*kdtree stuff of a cluster*/
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_cluster;
    pcl::PointCloud<pcl::PointXYZI>::Ptr remaining_pc_per_cluster (new pcl::PointCloud<pcl::PointXYZI>);

    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
      pcl::PointXYZI i_pt;
      i_pt.x = pc_original_z_up_->points[*pit].x;
      i_pt.y = pc_original_z_up_->points[*pit].y;
      i_pt.z = pc_original_z_up_->points[*pit].z;
      i_pt.intensity = intensity_cnt;
      current_remaining_pc->push_back(i_pt);

      /*Aggregate a cluster*/   
      remaining_pc_per_cluster->push_back(pc_original_z_up_->points[*pit]);

    } 
    intensity_cnt+=200;
    kdtree_cluster.setInputCloud (remaining_pc_per_cluster);
    map_kdtree_current_remaining_[it] = kdtree_cluster;
  }  

  *current_remaining_pc_plus_patch += *current_remaining_pc;

  for(auto i=patch_pcs_.begin();i!=patch_pcs_.end();i++){
    *current_remaining_pc_plus_patch += (*(*i));
  }

  pcl_conversions::toPCL(ros::Time::now(), current_remaining_pc_plus_patch->header.stamp);
  current_remaining_pc_plus_patch->header.frame_id = global_frame_;
  pub_op_current_remaining_pc_.publish(current_remaining_pc_plus_patch);
}

void Ground_PointCloud_Editor::funcSavePC(){

  pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_pc (new pcl::PointCloud<pcl::PointXYZ>);

  for (std::vector<pcl::PointIndices>::iterator it = clusters_.begin (); it != clusters_.end (); ++it)
  {
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
      pcl::PointXYZ pt;
      pt.x = pc_original_z_up_->points[*pit].x;
      pt.y = pc_original_z_up_->points[*pit].y;
      pt.z = pc_original_z_up_->points[*pit].z;
      remaining_pc->push_back(pt);
    } 
  }  

  /*Aggregate all patch*/
  pcl::PointCloud<pcl::PointXYZI>::Ptr aggregate_patches(new pcl::PointCloud<pcl::PointXYZI>);
  for(auto i=patch_pcs_.begin();i!=patch_pcs_.end();i++){
    *aggregate_patches += (*(*i));
  }

  /*type transform from XYZI into XYZ*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr aggregate_patches_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*aggregate_patches, *aggregate_patches_xyz);

  /*Combine patch and remaining pc*/
  *remaining_pc += *aggregate_patches_xyz;

  pcl_conversions::toPCL(ros::Time::now(), remaining_pc->header.stamp);
  remaining_pc->header.frame_id = global_frame_;
  pcl::io::savePCDFileASCII ("/tmp/ground.pcd", *remaining_pc);
}

void Ground_PointCloud_Editor::funcRemoveWalls(){ 

  /*
  iterate clusters and find normals which not fit ground characteristics (normal z is not pointing up)
  We only consider perpendicular walls, therefore the threshold is very strictly, z must small than 5 times x||y
  */
  int cluster_num = 0;
  for (auto it = clusters_.begin (); it != clusters_.end ();)
  { 
    double sum_nx = 0;
    double sum_ny = 0;
    double sum_nz = 0;
    double num_pt = 0;
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){

      sum_nx += normals_->points[*pit].normal_x;
      sum_ny += normals_->points[*pit].normal_y;
      sum_nz += normals_->points[*pit].normal_z;
      num_pt += 1;
    }

    sum_nx/=num_pt;
    sum_ny/=num_pt;
    sum_nz/=num_pt;
    if(fabs(sum_nz)<=5.*fabs(sum_nx) && fabs(sum_nz)<=5.*fabs(sum_ny)) // not z dominant
    {
      clusters_.erase(it);
    }
    else{
      it++;
    }
    ROS_DEBUG("Num: %d, x: %f, y: %f, z: %f", cluster_num, sum_nx, sum_ny, sum_nz);
    cluster_num++;
  }  
  /*Wall is removed, iterators are changed, therefore, we clear all and reset*/
  selected_clusters_.clear();
  updateSelectedCluster();
  pubCurrentRemainingPC();

}

void Ground_PointCloud_Editor::funcLastStep(){
  
  if(clusters_op_steps_.size()<1){
    return;
  }
  auto a_tmp_pair = clusters_op_steps_.back();
  clusters_ = a_tmp_pair.first;
  patch_pcs_ = a_tmp_pair.second;
  clusters_op_steps_.pop_back();
  pubCurrentRemainingPC(); 
  
}

void Ground_PointCloud_Editor::funcDeleteSelectedClusters(){

  funcQueueOperation();

  std::vector <pcl::PointIndices> clusters_tmp;

  for(auto cit=clusters_.begin();cit!=clusters_.end();cit++){
    if(selected_clusters_.find(cit) == selected_clusters_.end()){
      clusters_tmp.push_back(*cit);
    }
  }
  clusters_ = clusters_tmp;
  selected_clusters_.clear();
  updateSelectedCluster();//make selection empty, so we can make the selection in rviz disappeared
  pubCurrentRemainingPC();  
}

void Ground_PointCloud_Editor::funcDeleteAggregatedPoints(){

  funcQueueOperation();

  for(unsigned int i = 0; i < pc_aggregated_->size(); i++){
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    /*Test1: Check if the point is in the patche, otherwise it is in clusters.*/

    /*Create a flag for boosting*/
    bool is_in_patch = false;
    /*Find which patch the point located in*/
    for(auto patch_it=patch_pcs_.begin(); patch_it!=patch_pcs_.end(); patch_it++){
      /*Create a kdtree for a patch*/
      pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_patched;
      if(!(*patch_it)->points.size()>0)//Since patch may all be removed
        continue;
      kdtree_patched.setInputCloud(*patch_it);
      if(!is_in_patch && kdtree_patched.radiusSearch (pc_aggregated_->points[i], 0.03, pointIdxRadiusSearch, pointRadiusSquaredDistance)>=1){
        /*Found the patch, now look for the index*/
        for (auto patch_pc_it = (*patch_it)->points.begin(); patch_pc_it!=(*patch_it)->points.end();){
          float x_diff = pc_aggregated_->points[i].x-(*patch_pc_it).x;
          float y_diff = pc_aggregated_->points[i].y-(*patch_pc_it).y;
          float z_diff = pc_aggregated_->points[i].z-(*patch_pc_it).z;
          if(fabs(x_diff)+fabs(y_diff)+fabs(z_diff)<0.01){
            (*patch_it)->points.erase(patch_pc_it);
            is_in_patch = true;
            break;
          }
          else{
            patch_pc_it++;
          }
        }
      }      
    }
    
    if(is_in_patch)
      continue;

    /*Test2: Find which cluster the aggregate point locates in*/
    for(auto it=map_kdtree_current_remaining_.begin();it!=map_kdtree_current_remaining_.end();it++){
      if((*it).second.radiusSearch (pc_aggregated_->points[i], 0.03, pointIdxRadiusSearch, pointRadiusSquaredDistance)>=1){
        /*Found the cluster, now look for the index*/
        for (std::vector<int>::const_iterator pit = (*it).first->indices.begin (); pit != (*it).first->indices.end (); ++pit){
          float x_diff = pc_aggregated_->points[i].x-pc_original_z_up_->points[*pit].x;
          float y_diff = pc_aggregated_->points[i].y-pc_original_z_up_->points[*pit].y;
          float z_diff = pc_aggregated_->points[i].z-pc_original_z_up_->points[*pit].z;
          if(fabs(x_diff)+fabs(y_diff)+fabs(z_diff)<0.01){
            (*it).first->indices.erase(pit);
            break;
          }

        }

      }
    }

  }

  pc_aggregated_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  pc_aggregated_->header.frame_id = global_frame_;
  pub_op_current_aggregated_pc_.publish(pc_aggregated_);
  updateSelectedCluster(); //update clusters
  pubCurrentRemainingPC();  

}


void Ground_PointCloud_Editor::funcPatchPlanar(){

  funcQueueOperation();

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (pc_aggregated_);
  seg.segment (*inliers, *coefficients);
  // rasanc Ax+By+Cz+D = 0
  ROS_DEBUG("%f, %f, %f, %f", coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);
  /*Find min-max xy*/
  pcl::PointXYZI min_pt, max_pt;
  pcl::getMinMax3D(*pc_aggregated_, min_pt, max_pt);

  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(pc_aggregated_);

  
  pcl::PointCloud<pcl::PointXYZI>::Ptr patch_pc (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr aggregated_plus_patch_pc (new pcl::PointCloud<pcl::PointXYZI>);
  *aggregated_plus_patch_pc += *pc_aggregated_;

  for(double xi=min_pt.x; xi<max_pt.x; xi+=0.05){
    for(double yi=min_pt.y;yi<max_pt.y; yi+=0.05){
      pcl::PointXYZI ipt;
      ipt.x = xi;
      ipt.y = yi;      
      ipt.z = (-coefficients->values[3]-coefficients->values[0]*xi-coefficients->values[1]*yi)/coefficients->values[2];
      ipt.intensity = 10000;
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;
      if(kdtree.radiusSearch (ipt, 0.3, pointIdxRadiusSearch, pointRadiusSquaredDistance)<1)
      { 
        patch_pc->push_back(ipt);
        *aggregated_plus_patch_pc += *patch_pc;
        kdtree.setInputCloud(aggregated_plus_patch_pc);
      }
    }
  }

  pc_aggregated_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  pc_aggregated_->header.frame_id = global_frame_;
  pub_op_current_aggregated_pc_.publish(pc_aggregated_);
  patch_pcs_.push_back(patch_pc);
  pubCurrentRemainingPC();
  ROS_WARN("Patch done.");

}

void Ground_PointCloud_Editor::cbKeyBoardPoints(const std_msgs::String::ConstPtr& msg){

  if( strstr(msg->data.c_str(),"p") || strstr(msg->data.c_str(),"P") ){
    if(pc_aggregated_->points.size()<3){
      ROS_WARN("Detect %s, selected points are less than 3. Selected more points.",msg->data.c_str());
      return;
    }
    ROS_WARN("Detect %s, Patch the area.",msg->data.c_str());
    funcPatchPlanar();
  }
  else if( strstr(msg->data.c_str(),"D") || strstr(msg->data.c_str(),"d") ){
    ROS_WARN("Detect %s, delete selected points.",msg->data.c_str());
    funcDeleteAggregatedPoints();
  }
  else if( strstr(msg->data.c_str(),"W") || strstr(msg->data.c_str(),"w") ){
    ROS_WARN("Detect %s, delete wall clusters.",msg->data.c_str());
    funcRemoveWalls();
  }
  else if( strstr(msg->data.c_str(),"R") || strstr(msg->data.c_str(),"r") ){
    ROS_WARN("Detect %s, reset clusters.",msg->data.c_str());
    funcResetClusters();
  }
  else if( strstr(msg->data.c_str(),"K") || strstr(msg->data.c_str(),"k") ){
    ROS_WARN("Detect %s, save clusters.",msg->data.c_str());
    funcSavePC();
  }
  else if( strstr(msg->data.c_str(),"Z") || strstr(msg->data.c_str(),"z") ){
    ROS_WARN("Detect %s, back to last step.",msg->data.c_str());
    funcLastStep();
  }

  
}

void Ground_PointCloud_Editor::cbKeyBoardClusters(const std_msgs::String::ConstPtr& msg){

  if( strstr(msg->data.c_str(),"D") || strstr(msg->data.c_str(),"d") ){
    ROS_WARN("Detect %s, delete selected clusters.",msg->data.c_str());
    funcDeleteSelectedClusters();
  }
  else if( strstr(msg->data.c_str(),"W") || strstr(msg->data.c_str(),"w") ){
    ROS_WARN("Detect %s, delete wall clusters.",msg->data.c_str());
    funcRemoveWalls();
  }
  else if( strstr(msg->data.c_str(),"R") || strstr(msg->data.c_str(),"r") ){
    ROS_WARN("Detect %s, reset clusters.",msg->data.c_str());
    funcResetClusters();
  }
  else if( strstr(msg->data.c_str(),"K") || strstr(msg->data.c_str(),"k") ){
    ROS_WARN("Detect %s, save clusters.",msg->data.c_str());
    funcSavePC();
  }
  else if( strstr(msg->data.c_str(),"Z") || strstr(msg->data.c_str(),"z") ){
    ROS_WARN("Detect %s, back to last step.",msg->data.c_str());
    funcLastStep();
  }
}

void Ground_PointCloud_Editor::funcResetClusters(){

  /*
  We transform pcl::PointXYZ to pcl::PointXYZI as our format,
  We stick to XYZI format in this application.
  */
  
  clusters_.clear();
  clusters_op_steps_.clear();


  pcl::search::Search<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
  normals_ = pcl::PointCloud <pcl::Normal>::Ptr (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (pc_original_z_up_);
  normal_estimator.setKSearch (30);
  normal_estimator.compute (*normals_);

  pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
  reg.setMinClusterSize (20);
  reg.setMaxClusterSize (10000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (pc_original_z_up_);
  reg.setInputNormals (normals_);
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (2.0);

  
  reg.extract (clusters_);
  
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl_conversions::toPCL(ros::Time::now(), colored_cloud->header.stamp);
  colored_cloud->header.frame_id = global_frame_;
  pub_raw_transformed_pc_.publish(colored_cloud);
  
  selected_clusters_.clear();
  updateSelectedCluster();
  pubCurrentRemainingPC();

}

void Ground_PointCloud_Editor::cbRvizSelectedPoints(const sensor_msgs::PointCloud2::ConstPtr& msg){

  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_msg_raw(new pcl::PointCloud<pcl::PointXYZ>);  
  pcl::fromROSMsg(*msg, *pc_msg_raw);
  pcl::copyPointCloud(*pc_msg_raw, *pc_aggregated_);
  pcl_conversions::toPCL(ros::Time::now(), pc_aggregated_->header.stamp);
  pc_aggregated_->header.frame_id = global_frame_;
  pub_op_current_aggregated_pc_.publish(pc_aggregated_);

}

void Ground_PointCloud_Editor::cbRvizSelectedClusters(const sensor_msgs::PointCloud2::ConstPtr& msg){

  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_msg_raw(new pcl::PointCloud<pcl::PointXYZ>);  
  pcl::PointCloud<pcl::PointXYZI>::Ptr rviz_selected_points (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *pc_msg_raw);
  pcl::copyPointCloud(*pc_msg_raw, *rviz_selected_points);

  selected_clusters_.clear();

  for(unsigned int i = 0; i < rviz_selected_points->size(); i++){
    for(auto it=map_kdtree_current_remaining_.begin();it!=map_kdtree_current_remaining_.end();it++){

      if(selected_clusters_.find((*it).first) != selected_clusters_.end())
        continue;

      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;
      if((*it).second.radiusSearch (rviz_selected_points->points[i], 0.05, pointIdxRadiusSearch, pointRadiusSquaredDistance)>=1){
        selected_clusters_.insert((*it).first);
      }
    }

  }
  ROS_DEBUG("Number of clusters: %lu", selected_clusters_.size());
  updateSelectedCluster();

}

void Ground_PointCloud_Editor::cbMapcloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_msg_raw(new pcl::PointCloud<pcl::PointXYZ>);  
  pc_original_z_up_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *pc_msg_raw);

  // transform into type XYZI
  pcl::copyPointCloud(*pc_msg_raw, *pc_original_z_up_);

  /*  Using a Affine3f
    This method is easier and less error prone
  */
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

  // Define a translation of 0.0 meters on the axis.
  transform_2.translation() << translate_x_, translate_y_, translate_z_;

  // The same rotation matrix as before; theta radians around X axis
  if(fabs(rotate_around_x_)>0.01)
    transform_2.rotate (Eigen::AngleAxisf (rotate_around_x_, Eigen::Vector3f::UnitX()));
  if(fabs(rotate_around_y_)>0.01)
    transform_2.rotate (Eigen::AngleAxisf (rotate_around_y_, Eigen::Vector3f::UnitY()));
  if(fabs(rotate_around_z_)>0.01)
    transform_2.rotate (Eigen::AngleAxisf (rotate_around_z_, Eigen::Vector3f::UnitZ()));  

  // apply transform
  pcl::transformPointCloud (*pc_original_z_up_, *pc_original_z_up_, transform_2);



  global_frame_ = msg->header.frame_id;

  pub_raw_pc_.publish(pc_original_z_up_);

  funcResetClusters();
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ground_pointcloud_editor_node");


  ros::NodeHandle nh_;
  ros::NodeHandle pnh_("~");
  Ground_PointCloud_Editor GED = Ground_PointCloud_Editor(nh_,pnh_);

  ros::spin();

  return 0;
}