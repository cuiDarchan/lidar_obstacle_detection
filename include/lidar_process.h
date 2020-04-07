/**
 * @file: lidar_process.h
 * @author: cuiDarchan
 * @date: 2019.10.30
 */

#pragma once

// ros
#include <ros/ros.h>

// std
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

// pcl
#include <pcl/common/angles.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h> //半径滤波器头文件
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>

// eigen库
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

// tf
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// map_info
#define map_border_x 683100.0
#define map_border_y 3110700.0
#define x_min -25
#define y_min -35
#define z_min -3
#define x_max 25
#define y_max 50
#define z_max 5
#define map_resolution 51.2

// vehicle range
#define car_left -1.6
#define car_right 1.6
#define car_forward 2.0
#define car_behind -14.0
#define car_h 3
#define K 2
#define K_threshold_ 0.8

class LidarProcess {

public:
  LidarProcess(ros::NodeHandle &nh);

  ~LidarProcess();

private:
  void PreProcess(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);

  void CalculateKNNMap(pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_ptr,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr knn_cloud_ptr);

  void LoadMap(double map_x, double map_y,
               pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_ptr);

  void GetPosefromTF(double timestamp,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud);

  void GetObstacle(pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr knn_cloud_);

  void ProcessCallback(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr);

  // sub
  ros::Subscriber sub_point_cloud_;

  // pub
  ros::Publisher pub_pc_;
  ros::Publisher pub_pc_map_;
  ros::Publisher pub_obstacles_;

  std::string lidar_topic;
  std::string map_path;
  std::string lidar_frame_id;

  // lidar_map_index
  int global_x_ = 0;
  int global_y_ = 0;

  // tf
  Eigen::Affine3d pose_;
  Eigen::Matrix4d novatel2world_trans_;

  // point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;     // map point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud_; // obstacle point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr knn_cloud_;    // knn_map point cloud
};