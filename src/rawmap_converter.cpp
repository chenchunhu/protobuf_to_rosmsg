// 标准库相关
#include <fstream>

// ros相关
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <pcl/pcl_macros.h>

// proto 自动生成的头文件
#include "message_map.pb.h"

#include "converter.h"
#include "rawmap_converter.h"

namespace seer {

RawMapConverter::RawMapConverter(ros::NodeHandle& nh) : nh_(nh) {}

void RawMapConverter::convert_to_rawmap(const std::string& file_path, const std::string& output_rawmap_path) {
  // 读取rosbag
  rosbag::Bag bag;
  printf("open bag %s ...\n", file_path.c_str());
  bag.open(file_path, rosbag::bagmode::Read);

  // 打开失败
  if (!bag.isOpen()) {
    printf("open bag %s falied!!\n", file_path.c_str());
    ros::shutdown();
    exit(0);
  }

  std::vector<double> t_imu2base;
  tf::Quaternion q_imu2base;
  std::vector<double> t_lidar2base;
  tf::Quaternion q_lidar2base;
  Converter::get_extrinsic(t_imu2base, q_imu2base, t_lidar2base, q_lidar2base);

  std::array<double, 3> rpy_lidar2base{};
  tf::Matrix3x3(q_lidar2base).getRPY(rpy_lidar2base[0], rpy_lidar2base[1], rpy_lidar2base[2]);

  // 获取想要的topics
  auto imu_topic = nh_.param<std::string>("imu_topic", "/imu_data");
  printf("imu_topic: %s\n", imu_topic.c_str());
  auto lidar_topic = nh_.param<std::string>("lidar_topic", "/lidar_data");
  printf("lidar_topic: %s\n", lidar_topic.c_str());
  auto partial_convert_ratio = nh_.param<double>("partial_convert_ratio", 1.0);
  printf("partial_convert_ratio: %f\n", partial_convert_ratio);

  auto input_lidar_type = nh_.param<int>("input_lidar_type", 0);
  printf("input_lidar_type: %d\n", input_lidar_type);

  rosbag::View view(bag, rosbag::TopicQuery(imu_topic));
  auto b_time = view.getBeginTime();
  auto e_time = view.getEndTime();
  // 计算新的时间
  ros::Time new_e_time;
  new_e_time.fromSec(b_time.toSec() + (e_time.toSec() - b_time.toSec()) * partial_convert_ratio);
  printf("bag time range:%.3f - %.3f, new end time: %.3f, size: %d\n", b_time.toSec(), e_time.toSec(),
         new_e_time.toSec(), view.size());
  rosbag::View imu_filtered_view(bag, rosbag::TopicQuery(imu_topic), b_time, new_e_time);
  b_time = imu_filtered_view.getBeginTime();
  e_time = imu_filtered_view.getEndTime();
  printf("imu filtered bag time range:%.3f - %.3f, size: %d\n", b_time.toSec(), e_time.toSec(),
         imu_filtered_view.size());

  // 转换出的地图数据
  rbk::protocol::Message_MapLog map_log;
  map_log.set_laser_name("converted");
  // 配置安装参数
  map_log.set_laser_pos_x(t_lidar2base[0]);
  map_log.set_laser_pos_y(t_lidar2base[1]);
  map_log.set_laser_install_height(t_lidar2base[2]);
  map_log.set_laser_install_roll(rpy_lidar2base[0]);
  map_log.set_laser_install_pitch(rpy_lidar2base[1]);
  map_log.set_laser_install_yaw(rpy_lidar2base[2]);

  // 统计转换进度，每秒钟输出一次状态
  uint32_t tick_time{};
  uint32_t converted_cnt{0};
  // 打印进程函数
  auto print_process = [&tick_time, &converted_cnt](int total_cnt) {
    if (ros::Time::now().sec - tick_time >= 1) {
      tick_time = ros::Time::now().sec;
      printf("%d/%d\n", converted_cnt, total_cnt);
    }
  };

  map_log.mutable_imu_data()->Reserve(int(imu_filtered_view.size()));
  tick_time = ros::Time::now().sec;
  converted_cnt = 0;

  printf("converting imu data...\n");
  for (const auto& msg : imu_filtered_view) {
    auto imu_data = msg.instantiate<sensor_msgs::Imu>();
    if (imu_data != nullptr) {
      auto log_imu = map_log.add_imu_data();
      Converter::toImu(imu_data, log_imu);

      // 修改安装位置信息
      log_imu->mutable_install_info()->set_x(t_imu2base[0]);
      log_imu->mutable_install_info()->set_y(t_imu2base[1]);
      log_imu->mutable_install_info()->set_z(t_imu2base[2]);
      log_imu->mutable_install_info()->set_qx(q_imu2base.x());
      log_imu->mutable_install_info()->set_qy(q_imu2base.y());
      log_imu->mutable_install_info()->set_qz(q_imu2base.z());
      log_imu->mutable_install_info()->set_qw(q_imu2base.w());
      log_imu->mutable_install_info()->set_ssf(1.);
    }

    converted_cnt++;
    print_process(int(imu_filtered_view.size()));
  }

  rosbag::View lidar_filtered_view(bag, rosbag::TopicQuery(lidar_topic), b_time, new_e_time);
  b_time = lidar_filtered_view.getBeginTime();
  e_time = lidar_filtered_view.getEndTime();
  printf("lidar filtered bag time range:%.3f - %.3f, size: %d\n", b_time.toSec(), e_time.toSec(),
         lidar_filtered_view.size());

  map_log.mutable_log_data3d()->Reserve(int(lidar_filtered_view.size()));
  tick_time = ros::Time::now().sec;
  converted_cnt = 0;

  printf("converting lidar data...\n");
  for (const auto& msg : lidar_filtered_view) {
    auto lidar_data = msg.instantiate<sensor_msgs::PointCloud2>();
    if (lidar_data != nullptr) {
      auto log_data3d = map_log.add_log_data3d();
      Converter::toMapLogData3d(input_lidar_type, lidar_data, log_data3d);
    }

    converted_cnt++;
    print_process(int(lidar_filtered_view.size()));
  }

  bag.close();

  if (imu_filtered_view.size() + lidar_filtered_view.size() > 0) {
    printf("all data converted!!!\n");
  } else {
    printf("no data need to convert!!!\n");
    return;
  }

  // 保存文件
  std::ofstream fs;
  fs.open(output_rawmap_path, std::ios::out | std::ios::binary);

  if (!fs.is_open()) {
    printf("cannot open file %s!!!!!", output_rawmap_path.c_str());
    return;
  }

  printf("saving to rawmap...\n");
  if (map_log.SerializePartialToOstream(&fs)) {
    printf("rawmap size: %.3fMB\n", double(map_log.SpaceUsedLong()) / 1024. / 1024.);
    printf("rawmap saved to %s\n", output_rawmap_path.c_str());
  } else {
    printf("rawmap save failed!!!!\n");
  }
}

}  // namespace seer
