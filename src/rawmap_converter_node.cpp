#include <string>

// ros相关
#include <ros/ros.h>

#include "rawmap_converter.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "rawmap_converter");
  ros::NodeHandle nh("~");

  // 待转发的文件路径
  // std::string filename;

  // 判断是否从文件加载
  if (argc < 3) {
    ROS_ERROR("need input args >= 2");
    ros::shutdown();
    exit(0);
  }

  seer::RawMapConverter converter(nh);

  // 转为rosbag
  converter.convert_to_rawmap(std::string(argv[1]), std::string(argv[2]));

  return 0;
}