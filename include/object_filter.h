#ifndef _BOREALIS_MAPPING_OBJECT_FILTER_H_
#define _BOREALIS_MAPPING_OBJECT_FILTER_H_

#include "dynamic_object.h"

#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <memory>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/min_cut_segmentation.h>

typedef geometry_msgs::PoseWithCovarianceStamped ObjectPoseType;
typedef sensor_msgs::PointCloud2 PointCloudType;

                       


#endif //_BOREALIS_MAPPING_OBJECT_FILTER_H_