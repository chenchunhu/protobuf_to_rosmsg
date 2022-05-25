#pragma once

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

namespace rbk {
namespace protocol {
class Message_MapLogData3D;
}
}  // namespace rbk

namespace seer {
class Converter {
 public:
  /**
   * @brief 从配置中获取外参
   * @param t_imu2base
   * @param q_imu2base
   * @param t_lidar2base
   * @param q_lidar2base
   */
  static void get_extrinsic(std::vector<double>& t_imu2base, tf::Quaternion& q_imu2base,
                            std::vector<double>& t_lidar2base, tf::Quaternion& q_lidar2base);
  /**
   * @brief 将地图数据中的3d激光数据转为ros标准的PointCloud2格式
   * @param input_lidar_type 输出激光数据类型： 0: normal - 每个点的时间由offset构成
                                            1: helios - 每个点的时候是真实的时间，但是helios的时间需要额外处理
   * @param data 地图数据中一帧激光数据
   * @return 转换好的PointCloud2格式激光数据
   */
  static void toPointCloud2(int output_lidar_type, const rbk::protocol::Message_MapLogData3D& data,
                            sensor_msgs::PointCloud2& msg);

  static void parse_raw_data(const rbk::protocol::Message_MapLog& map_log, std::vector<sensor_msgs::PointCloud2>& msgs);

  /**
   * @brief 将地图数据中的imu据转为ros标准的Imu格式
   * @param data 地图数据中一帧imu数据
   * @param imu 转换出的Imu数据
   */
  static void toImu(const rbk::protocol::Message_IMU& data, sensor_msgs::Imu& imu);

  /**
   * @brief 将地图数据中的里程数据转为ros标准的Odometry格式
   * @param data 地图数据中一帧里程数据
   * @param odom 转换出的Odometry数据
   */
  static void toOdom(const rbk::protocol::Message_MapOdo& data, nav_msgs::Odometry& odom);

  /**
   * @brief 将ros标准的PointCloud2格式数据转为地图数据中的3d激光数据
   * @param input_lidar_type 输入激光数据类型： 0: normal - 每个点的时间由offset构成
                                            1: helios - 每个点的时候是真实的时间，但是helios的时间需要额外处理
   * @param msg 原始激光数据
   * @param data 地图数据中一帧激光数据
   */
  static void toMapLogData3d(int input_lidar_type, const sensor_msgs::PointCloud2::Ptr& msg,
                             rbk::protocol::Message_MapLogData3D* data);

  /**
   * @brief 将ros imu数据转为地图数据格式
   * @param imu
   * @param data
   */
  static void toImu(const sensor_msgs::Imu::Ptr& imu, rbk::protocol::Message_IMU* data);
};
}  // namespace seer
