#pragma once

#include <string>

namespace ros {
class NodeHandle;
}

namespace seer {
class RawMapConverter {
 public:
  /**
   * @brief 构造函数
   * @param nh
   */
  explicit RawMapConverter(ros::NodeHandle& nh);

  /**
   * @brief 将rosbag中的激光和imu数据转存为rawmap
   * @param file_path
   * @param output_rawmap_path
   */
  void convert_to_rawmap(const std::string& file_path, const std::string& output_rawmap_path);

 private:
  ros::NodeHandle& nh_;
};
}  // namespace seer