/**
 * @file: lidar_process.cpp
 * @author: cuiDarchan
 * @date: 2019.10.30
 */

#include "lidar_process.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr
    stored_cloud_(new pcl::PointCloud<pcl::PointXYZ>);
tf2_ros::Buffer tf2_buffer_;
tf2_ros::TransformListener *tf2_transform_listener_ = NULL;

LidarProcess::LidarProcess(ros::NodeHandle &nh) {

  tf2_transform_listener_ = new tf2_ros::TransformListener(tf2_buffer_, nh);
  // set params
  nh.param("lidar_topic", lidar_topic, std::string(""));
  nh.param("map_path", map_path, std::string(""));
  nh.param("lidar_frame_id", lidar_frame_id, std::string(""));
  // sub
  sub_point_cloud_ =
      nh.subscribe(lidar_topic, 10, &LidarProcess::ProcessCallback, this);
  // pub
  pub_pc_ = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud_frame", 1000);
  pub_pc_map_ =
      nh.advertise<sensor_msgs::PointCloud2>("/point_cloud_map", 1000);
  pub_obstacles_ =
      nh.advertise<sensor_msgs::PointCloud2>("/point_obstacles", 1000);
  ros::spin();
}

LidarProcess::~LidarProcess() {}

void LidarProcess::PreProcess(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr tem_cloud_(
      new pcl::PointCloud<pcl::PointXYZ>);

  // 1. Get point cloud of map region
  pcl::CropBox<pcl::PointXYZ> box_filter;
  box_filter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
  box_filter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
  box_filter.setNegative(false);
  box_filter.setInputCloud(pointcloud_);
  box_filter.filter(*tem_cloud_);

  // 2. Crop vehicle
  std::vector<int> index_car;
  for (int t = 0; t < tem_cloud_->size(); t++) {
    if (tem_cloud_->points[t].x < car_right &&
        tem_cloud_->points[t].x > car_left &&
        tem_cloud_->points[t].y < car_forward &&
        tem_cloud_->points[t].y > car_behind) {
      index_car.push_back(t);
    }
  }
  boost::shared_ptr<std::vector<int>> index_car_ptr =
      boost::make_shared<std::vector<int>>(index_car);
  pcl::ExtractIndices<pcl::PointXYZ> extract_car;
  extract_car.setInputCloud(tem_cloud_);
  extract_car.setIndices(index_car_ptr);
  extract_car.setNegative(true);
  extract_car.filter(*pointcloud_);
}

void LidarProcess::CalculateKNNMap(
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_point_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr knn_point_cloud) {

  // Match KNN map block
  pcl::CropBox<pcl::PointXYZ> box_filter;
  box_filter.setMin(Eigen::Vector4f(x_min - 2, y_min - 2, z_min, 1.0));
  box_filter.setMax(Eigen::Vector4f(x_max + 2, y_max + 2, z_max, 1.0));
  box_filter.setNegative(false);
  box_filter.setInputCloud(map_point_cloud);
  box_filter.filter(*knn_point_cloud);
}

void LidarProcess::LoadMap(double map_x, double map_y,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_) {

  int x = std::floor(map_x / map_resolution);
  int y = std::floor(map_y / map_resolution);
  pcl::PointCloud<pcl::PointXYZ> center_cloud_;

  // Load map file
  if (global_x_ != x || global_y_ != y) {
    for (int i = x - 1; i <= x + 1; i++) {
      for (int j = y - 1; j <= y + 1; j++) {
        std::string filepath_ =
            map_path + std::to_string(i) + "_" + std::to_string(j) + ".pcd";
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(filepath_, center_cloud_) !=
            -1) {
          // transform
          int pt_size = center_cloud_.size();
          for (int pt = 0; pt < pt_size; pt++) {
            center_cloud_.points[pt].x += i * map_resolution;
            center_cloud_.points[pt].x -= map_border_x;
            center_cloud_.points[pt].y += j * map_resolution;
            center_cloud_.points[pt].y -= map_border_y;
          }
          *map_cloud_ += center_cloud_;
        }
      }
    }
    global_x_ = x;
    global_y_ = y;
    *stored_cloud_ = *map_cloud_;
  } else {
    *map_cloud_ = *stored_cloud_;
  }
}

void LidarProcess::GetPosefromTF(
    double timestamp, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud) {

  ros::Time query_time(timestamp);
  std::string errMsg;
  geometry_msgs::TransformStamped stamped_transform;
  if (!tf2_buffer_.canTransform("world", "novatel", query_time,
                                ros::Duration(0.8), &errMsg)) {
    std::cout << "Can not find transform. " << std::endl;
  }

  try {
    stamped_transform =
        tf2_buffer_.lookupTransform("world", "novatel", query_time);
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
  }

  tf::transformMsgToEigen(stamped_transform.transform, pose_);
  novatel2world_trans_ = Eigen::Matrix4d::Identity();
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      novatel2world_trans_(i, j) = pose_(i, j);
    }
  }
  novatel2world_trans_(0, 3) = novatel2world_trans_(0, 3) - map_border_x;
  novatel2world_trans_(1, 3) = novatel2world_trans_(1, 3) - map_border_y;
}

void LidarProcess::GetObstacle(
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr,
    pcl::PointCloud<pcl::PointXYZ>::Ptr knn_cloud_) {

  std::vector<int> index;
  std::vector<int> pointIdxKNNSearch(K);
  std::vector<float> pointKNNSquaredDistance(K);
  // kdtree
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(knn_cloud_);
  for (uint pts = 0; pts < current_pc_ptr->size(); pts++) {
    pcl::PointXYZ searchPoint;
    if (std::isnan(current_pc_ptr->points[pts].x))
      continue;
    searchPoint.x = current_pc_ptr->points[pts].x;
    searchPoint.y = current_pc_ptr->points[pts].y;
    searchPoint.z = current_pc_ptr->points[pts].z;
    // KNN search, if exceeding the threshold, considered a dynamic target
    if (kdtree.nearestKSearch(searchPoint, K, pointIdxKNNSearch,
                              pointKNNSquaredDistance) > 0) {
      float sum = 0;
      for (int tem_i = 0; tem_i < K; tem_i++) {
        sum += pointKNNSquaredDistance[tem_i];
      }
      float avr_dis = sum / K;
      if (avr_dis > K_threshold_) {
        index.push_back(pts);
      }
    }
  }

  boost::shared_ptr<std::vector<int>> index_ptr =
      boost::make_shared<std::vector<int>>(index);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(current_pc_ptr);
  extract.setIndices(index_ptr);
  extract.setNegative(false);
  extract.filter(*object_cloud_);
}

void LidarProcess::ProcessCallback(
    const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr) {

  if (in_cloud_ptr->data.empty()) {
    return;
  }

  // init
  map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  object_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  knn_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);

  // 1. Get pose from transform
  const double kTimeStamp = in_cloud_ptr->header.stamp.toSec() + 0.1;
  GetPosefromTF(kTimeStamp, current_pc_ptr);

  // 2. Pre-process point cloud
  PreProcess(current_pc_ptr);
  
  // 3. Load map file
  double map_x = novatel2world_trans_(0, 3) + map_border_x;
  double map_y = novatel2world_trans_(1, 3) + map_border_y;
  LoadMap(map_x, map_y, map_cloud_);
  Eigen::Matrix4d transform_inv = novatel2world_trans_.inverse();

  pcl::PointCloud<pcl::PointXYZ>::Ptr imu_map_cloud(
      new pcl::PointCloud<pcl::PointXYZ>); // map_cloud in IMU coordinate
  pcl::transformPointCloud(*map_cloud_, *imu_map_cloud, transform_inv);
  
  // 4.Calculate lidar KNN map
  CalculateKNNMap(imu_map_cloud, knn_cloud_);
  
  // 5. Background removal
  GetObstacle(current_pc_ptr, knn_cloud_);

  // 6. Publish frame map
  sensor_msgs::PointCloud2 pub_pc_frame;
  sensor_msgs::PointCloud2 pub_map;
  sensor_msgs::PointCloud2 pub_obstacles;

  pcl::toROSMsg(*current_pc_ptr, pub_pc_frame);
  pcl::toROSMsg(*imu_map_cloud, pub_map);
  pcl::toROSMsg(*object_cloud_, pub_obstacles);

  pub_pc_frame.header.frame_id = lidar_frame_id;
  pub_map.header.frame_id = lidar_frame_id;
  pub_obstacles.header.frame_id = lidar_frame_id;

  pub_pc_.publish(pub_pc_frame);
  pub_pc_map_.publish(pub_map);
  pub_obstacles_.publish(pub_obstacles);
}