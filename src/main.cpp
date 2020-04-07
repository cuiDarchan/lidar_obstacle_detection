/**
 * @file: main.cpp
 * @author: cuiDarchan
 * @date: 2019.10.30
 */

#include "lidar_process.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_obstacle_detection");

    ros::NodeHandle nh;

    LidarProcess lidar_process(nh);
     
    return 0;
}

