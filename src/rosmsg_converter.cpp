// 标准库相关
#include <fstream>

// ros相关
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <pcl/pcl_macros.h>

// proto 自动生成的头文件
#include "message_map.pb.h"

#include "converter.h"
#include "rosmsg_converter.h"

namespace seer {

void RosMsgConverter::convert_to_rosbag(const std::string& file_path, const std::string& output_rosbag_path) {
  auto input_lidar_msg_type = nh_.param<int>("input_lidar_msg_type", 0);
  printf("input_lidar_msg_type: %d\n", input_lidar_msg_type);
  auto output_lidar_type = nh_.param<int>("output_lidar_type", 0);
  printf("output_lidar_type: %d\n", output_lidar_type);

  // 加载解析文件
  std::ifstream fin;
  fin.open(file_path.c_str(), std::ios::in | std::ios::binary);
  if (!fin.is_open()) {
    printf("open file %s falied!!\n", file_path.c_str());
    ros::shutdown();
    exit(0);
  }

  // 写rosbag
  rosbag::Bag bag;
  bag.open(output_rosbag_path, rosbag::bagmode::Write);

  // 打开失败
  if (!bag.isOpen()) {
    printf("open bag %s falied!!\n", output_rosbag_path.c_str());
    ros::shutdown();
    exit(0);
  }

  // 用于读取参数
  auto use_defined_extrinsic = nh_.param<bool>("use_defined_extrinsic", false);
  std::vector<double> t_imu2base{0., 0., 0.};
  tf::Quaternion q_imu2base = tf::Quaternion::getIdentity();
  std::vector<double> t_lidar2base{0., 0., 0.};
  tf::Quaternion q_lidar2base = tf::Quaternion::getIdentity();
  if (use_defined_extrinsic) {
    printf("use_defined_extrinsic: %d\n", use_defined_extrinsic);
    Converter::get_extrinsic(t_imu2base, q_imu2base, t_lidar2base, q_lidar2base);
  }

  // 单位rad
  std::array<double, 3> rpy_lidar2base{0., 0., 0.};
  std::array<double, 3> rpy_imu2base{0., 0., 0.};

  // 开始解析
  printf("loading %s and parsing\n", file_path.c_str());
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

  // 提取激光，里程计，imu数据
  rbk::protocol::Message_MapLog map_log;
  if (map_log.ParseFromIstream(&fin)) {
    printf("find sensor data!sensor name: %s\n", map_log.laser_name().c_str());
    std::string info_str;
    ros::Time info_time;

    // 判断是否有imu数据
    if (!map_log.imu_data().empty()) {
      info_time.fromNSec(map_log.imu_data(0).header().data_nsec());
    } else {
      bag.close();
      fin.close();
      printf("no imu data!!\n");
      exit(0);
    }

    if (!use_defined_extrinsic) {
      // 提取出imu与lidar的外参信息
      // [roll, pitch, yaw] deg

      t_lidar2base[0] = map_log.laser_pos_x();
      t_lidar2base[1] = map_log.laser_pos_y();
      t_lidar2base[2] = map_log.laser_install_height();
      rpy_lidar2base[0] = map_log.laser_install_roll();
      rpy_lidar2base[1] = map_log.laser_install_pitch();
      rpy_lidar2base[2] = map_log.laser_install_yaw();
      // rpy_lidar2base[2] = RAD2DEG(map_log.laser_pos_z());

      printf("t_lidar2base(x, y, z): %.3f, %.3f, %.3f\n", t_lidar2base[0], t_lidar2base[1], t_lidar2base[2]);
      printf("rpy_lidar2base(deg): %.3f , %.3f, %.3f\n", rpy_lidar2base[0], rpy_lidar2base[1], rpy_lidar2base[2]);

      q_imu2base.setX(map_log.imu_data(0).install_info().qx());
      q_imu2base.setY(map_log.imu_data(0).install_info().qy());
      q_imu2base.setZ(map_log.imu_data(0).install_info().qz());
      q_imu2base.setW(map_log.imu_data(0).install_info().qw());
      t_imu2base[0] = map_log.imu_data(0).install_info().x();
      t_imu2base[1] = map_log.imu_data(0).install_info().y();
      t_imu2base[2] = map_log.imu_data(0).install_info().z();

      tf::Matrix3x3(q_imu2base).getRPY(rpy_imu2base[0], rpy_imu2base[1], rpy_imu2base[2]);

      printf("t_imu2base(x, y, z): %.3f, %.3f, %.3f\n", t_imu2base[0], t_imu2base[1], t_imu2base[2]);
      printf("rpy_imu2base(deg): %.3f , %.3f, %.3f\n", RAD2DEG(rpy_imu2base[0]), RAD2DEG(rpy_imu2base[1]),
             RAD2DEG(rpy_imu2base[2]));

    } else {
      tf::Matrix3x3(q_lidar2base).getRPY(rpy_lidar2base[0], rpy_lidar2base[1], rpy_lidar2base[2]);
      tf::Matrix3x3(q_imu2base).getRPY(rpy_imu2base[0], rpy_imu2base[1], rpy_imu2base[2]);
    }

    char buff[512];
    snprintf(buff, sizeof(buff),
             "t_lidar2base(x, y, z): (%.3f, %.3f, %.3f), rpy_lidar2base(deg): (%.3f, %.3f, %.3f), t_imu2base(x, y, "
             "z): (%.3f, %.3f, %.3f), rpy_imu2base(deg): (%.3f, %.3f, %.3f)",
             t_lidar2base[0], t_lidar2base[1], t_lidar2base[2], RAD2DEG(rpy_lidar2base[0]), RAD2DEG(rpy_lidar2base[1]),
             RAD2DEG(rpy_lidar2base[2]), t_imu2base[0], t_imu2base[1], t_imu2base[2], RAD2DEG(rpy_imu2base[0]),
             RAD2DEG(rpy_imu2base[1]), RAD2DEG(rpy_imu2base[2]));
    info_str.assign(buff);

    std_msgs::String info;
    info.data.assign(info_str);
    bag.write("/info_converted", info_time, info);
    printf("/info_converted: %s\n", info_str.c_str());

    // imu
    {
      printf("converting imu, %d in total...\n", map_log.imu_data_size());

      tick_time = ros::Time::now().sec;
      converted_cnt = 0;
      // imu数据
      sensor_msgs::Imu imu;

      printf("imu ssf: %f\n", map_log.imu_data(0).install_info().ssf());
      if (map_log.imu_data(0).install_info().ssf() < 0.01) {
        printf(" !!!!!! ssf: %f!!\n", map_log.imu_data(0).install_info().ssf());
      }

      for (const auto& item : map_log.imu_data()) {
        seer::Converter::toImu(item, imu);

        // 写入rosbag
        bag.write("/imu/raw_data", imu.header.stamp, imu);
        converted_cnt++;

        // 状态打印
        print_process(map_log.imu_data_size());
      }
      printf("imu data converted!\n");
    }

    // 判断是否有3D点云数据
    if (!map_log.log_data3d().empty()) {
      printf("converting 3d lidar (%s) frames, %d in total...\n", map_log.laser_name().c_str(),
             map_log.log_data3d_size());
      // 设置保存的话题名称
      std::string lidar_points_topic = "/converted_points";
      tick_time = ros::Time::now().sec;
      converted_cnt = 0;

      if (input_lidar_msg_type == 0) {
        printf("parse_raw_data...\n");
        std::vector<sensor_msgs::PointCloud2> lidar_msgs;
        seer::Converter::parse_raw_data(map_log, lidar_msgs);

        for (const auto& msg : lidar_msgs) {
          // 写入rosbag
          bag.write(lidar_points_topic, msg.header.stamp, msg);
        }
        printf("parse ok! %zu frames\n", lidar_msgs.size());

      } else {
        for (const auto& item : map_log.log_data3d()) {
          // 转为rosmsg
          sensor_msgs::PointCloud2 lidar_msg;
          seer::Converter::toPointCloud2(output_lidar_type, item, lidar_msg);
          // printf("timestamp:%.6f, timeoffset: %d, \n", item.timestamp(), item.timeoffset(0));

          lidar_msg.header.stamp.fromSec(item.timestamp());
          lidar_msg.header.seq = converted_cnt;
          lidar_msg.header.frame_id = "converted_lidar";

          // 写入rosbag
          bag.write(lidar_points_topic, lidar_msg.header.stamp, lidar_msg);
          converted_cnt++;

          // 状态打印
          print_process(map_log.log_data3d_size());
        }
      }

      printf("3d lidar frames converted!\n");
    } else {
      printf("no lidar data found!\n");
    }

    // 判断是否有里程数据
    if (!map_log.odometer().empty()) {
      printf("converting odom, %d in total...\n", map_log.odometer_size());

      tick_time = ros::Time::now().sec;
      converted_cnt = 0;

      // 里程计数据
      nav_msgs::Odometry odom;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";

      for (const auto& item : map_log.odometer()) {
        seer::Converter::toOdom(item, odom);
        odom.header.seq = converted_cnt;

        // 写入rosbag
        bag.write("/odom", odom.header.stamp, odom);
        converted_cnt++;

        // 状态打印
        print_process(map_log.odometer_size());
      }
      printf("odom data converted!\n");

    } else {
      printf("no odometer data found!\n");
    }

  } else {
    printf("no data found!\n");
  }

  fin.close();
  bag.close();
  printf("all data parsed ok!!! %s\n", output_rosbag_path.c_str());
}
}  // namespace seer
