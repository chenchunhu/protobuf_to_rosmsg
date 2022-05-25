#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// 带时间的信息的点云
struct PointXYZIRT {
  PCL_ADD_POINT4D;
  float intensity;    // 反射率
  uint16_t ring;      // 当前数据点所在的线数
  float time_offset;  // 当前点的偏移时间
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                                   std::uint16_t, ring, ring)(float, time_offset, time_offset))

namespace robosense_ros {
struct Point {
  PCL_ADD_POINT4D;
  uint8_t intensity;
  uint16_t ring = 0;
  double timestamp = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
}  // namespace robosense_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(robosense_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)(
                                      std::uint16_t, ring, ring)(double, timestamp, timestamp))