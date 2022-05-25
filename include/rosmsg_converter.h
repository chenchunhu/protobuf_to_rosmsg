#pragma once

#include <string>

namespace ros {
class NodeHandle;
}

namespace seer {
class RosMsgConverter {
 public:
  /**
   * @brief 构造函数
   * @param nh
   */
  explicit RosMsgConverter(ros::NodeHandle& nh) : nh_(nh){};

  /**
  * @brief 将msg文件中的传感器数据提取出来保存为rosbag
  * @param file_path
  * @param output_rosbag_path
  */
  void convert_to_rosbag(const std::string& file_path, const std::string& output_rosbag_path) ;

 private:
  ros::NodeHandle& nh_;
};
}  // namespace seer