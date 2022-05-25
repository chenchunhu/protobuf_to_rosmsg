// ros 相关
#include <ros/time.h>  // NOLINT
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_datatypes.h>

// pcl
#include <pcl/pcl_macros.h>
#include <pcl/common/transforms.h>

// proto 相关
#include "message_map.pb.h"

#include "converter.h"
#include "utils.h"

namespace seer {

static void to_helios(const rbk::protocol::Message_MapLogData3D& data, sensor_msgs::PointCloud2& msg) {
  pcl::PointCloud<robosense_ros::Point> pc;

  // 先申请足够的空间
  pc.points.reserve(data.x_size());
  pc.is_dense = true;

  int pts_cnt = 0;

  // 逐个转换数据
  for (int i = 0; i < data.x_size(); i++) {
    // 判断一个就够了
    if (std::isinf(data.x(i)) || std::isnan(data.x(i))) {
      continue;
    }
    // 填充数据
    robosense_ros::Point p{};
    p.x = data.x(i);
    p.y = data.y(i);
    p.z = data.z(i);
    p.intensity = data.intensity(i);
    p.ring = data.ring(i);
    // 每个点的真实时间，不是偏移时间
    p.timestamp = data.timeoffset(i) / 1e9 + data.timestamp();
    pts_cnt++;
    pc.points.push_back(p);
  }

  pc.width = pts_cnt;
  pc.height = 1;

  // 转为rosmsg
  pcl::toROSMsg(pc, msg);

  // printf("in size: %d, out size: %zu\n", data.x_size(), pc.size());
}

static void to_normal(const rbk::protocol::Message_MapLogData3D& data, sensor_msgs::PointCloud2& msg) {
  pcl::PointCloud<PointXYZIRT> pc;

  // 先申请足够的空间
  pc.points.reserve(data.x_size());
  pc.is_dense = true;

  int pts_cnt = 0;

  // 逐个转换数据
  for (int i = 0; i < data.x_size(); i++) {
    // 判断一个就够了
    if (std::isinf(data.x(i)) || std::isnan(data.x(i))) {
      continue;
    }
    // 填充数据
    PointXYZIRT p{};
    p.x = data.x(i);
    p.y = data.y(i);
    p.z = data.z(i);
    p.intensity = float(data.intensity(i));
    p.ring = data.ring(i);
    // 每个点的真实时间，不是偏移时间
    p.time_offset = data.timeoffset(i) / 1e9;
    pts_cnt++;
    pc.points.push_back(p);
  }

  pc.width = pts_cnt;
  pc.height = 1;

  // 转为rosmsg
  pcl::toROSMsg(pc, msg);

  printf("in size: %d, out size: %zu\n", data.x_size(), pc.size());
}

void Converter::parse_raw_data(const rbk::protocol::Message_MapLog& map_log,
                               std::vector<sensor_msgs::PointCloud2>& msgs) {
  typedef pcl::PointXYZINormal PointType;
  typedef pcl::PointCloud<PointType> PointCloudXYZIN;

  double last_lidar_timestamp{-1.};
  msgs.clear();

  for (int k = 0; k < map_log.log_data3d_size(); ++k) {
    const rbk::protocol::Message_MapLogData3D& log3d = map_log.log_data3d(k);
    PointCloudXYZIN::Ptr cloud_ptr(new PointCloudXYZIN());

    double packet_time = log3d.timestamp();
    if (packet_time < last_lidar_timestamp) {
      printf("lidar data loop back, clear buffer!!!!\n");
    }

    last_lidar_timestamp = packet_time;

    unsigned int half_beam = map_log.azimuthcorrection_size() / 2;

    for (int j = 0; j < map_log.azimuthcorrection_size(); j++) {
      for (int i = 0; i < log3d.data_size(); ++i) {
        float first_azimuth = log3d.firstazimuth(i) + map_log.azimuthcorrection(j);
        float second_azimuth = log3d.secondazimuth(i) + map_log.azimuthcorrection(j);
        float vertical_angle = map_log.verticalcorrection(j);
        const std::string& data_string = log3d.data(i);
        uint32_t time_offset = log3d.timeoffset(i);
        uint32_t point_time_offset = time_offset;
        float first_cos_Azimuth = std::cos(first_azimuth);
        float first_sin_Azimuth = std::sin(first_azimuth);
        float second_cos_Azimuth = std::cos(second_azimuth);
        float second_sin_Azimuth = std::sin(second_azimuth);
        float cosVertical = std::cos(vertical_angle);
        float sinVertical = std::sin(vertical_angle);
        unsigned short dis = *(unsigned short*)(data_string.c_str() + j * 3);
        float distanceM;
        if (map_log.lasertype() == 1 || map_log.lasertype() == 2) {
          auto dist = ((((unsigned short)(dis)&0xFF) << 8) | (((unsigned short)(dis)&0xFF00) >> 8));
          distanceM = (float)dist * map_log.factor();
          if (map_log.lasertype() == 1) {
            if (j < 16)
              point_time_offset += j * 2.8 * 1000;
            else
              point_time_offset += (55.5 + (j - 16) * 2.8) * 1000;
          } else {
            point_time_offset += j * 1.45 * 1000;
          }
        } else {
          distanceM = (float)dis * map_log.factor();
        }
        unsigned char intensity = *(unsigned char*)(data_string.c_str() + j * 3 + 2);
        float xy_distance = distanceM * cosVertical;
        float x, y, z;
        if (j < half_beam) {
          x = xy_distance * first_cos_Azimuth;
          y = -xy_distance * first_sin_Azimuth;
          z = distanceM * sinVertical;
        } else {
          x = xy_distance * second_cos_Azimuth;
          y = -xy_distance * second_sin_Azimuth;
          z = distanceM * sinVertical;
        }

        PointType p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.intensity = intensity;

        // 每个点的偏移时间offset time
        p.curvature = static_cast<float>(point_time_offset / 1e9);
        cloud_ptr->points.push_back(p);
      }
    }

    if (!cloud_ptr->empty()) {
      sensor_msgs::PointCloud2 msg;
      // 转为rosmsg
      pcl::toROSMsg(*cloud_ptr, msg);

      msg.header.stamp.fromSec(packet_time);
      msg.header.seq = k;
      msg.header.frame_id = "converted_lidar";

      msgs.emplace_back(msg);

    }
  }
}

void Converter::get_extrinsic(std::vector<double>& t_imu2base, tf::Quaternion& q_imu2base,
                              std::vector<double>& t_lidar2base, tf::Quaternion& q_lidar2base) {
  // 用于读取参数
  ros::NodeHandle nh;
  // imu到车体的位姿变换 平移向量
  t_imu2base = nh.param<std::vector<double>>("t_imu2base", {0, 0, 0});
  printf("t_imu2base(x, y, z): %.3f, %.3f, %.3f\n", t_imu2base[0], t_imu2base[1], t_imu2base[2]);
  // imu到车体的位姿变换 旋转矩阵(YPR) [roll, pitch, yaw] 单位：deg
  auto rpy_imu2base = nh.param<std::vector<double>>("rpy_imu2base", {0, 0, 0});
  printf("rpy_imu2base(deg): %.3f , %.3f, %.3f\n", rpy_imu2base[0], rpy_imu2base[1], rpy_imu2base[2]);

  q_imu2base.setRPY(DEG2RAD(rpy_imu2base[0]), DEG2RAD(rpy_imu2base[1]), DEG2RAD(rpy_imu2base[2]));
  printf("q_imu2base(x, y, z, w): %.3f , %.3f, %.3f, %.3f\n", q_imu2base.x(), q_imu2base.y(), q_imu2base.z(),
         q_imu2base.w());

  // imu到车体的位姿变换 平移向量
  t_lidar2base = nh.param<std::vector<double>>("t_lidar2base", {0, 0, 0});
  printf("t_lidar2base(x, y, z): %.3f, %.3f, %.3f\n", t_lidar2base[0], t_lidar2base[1], t_lidar2base[2]);
  // imu到车体的位姿变换 旋转矩阵(YPR) [roll, pitch, yaw] 单位：deg
  auto rpy_lidar2base = nh.param<std::vector<double>>("rpy_lidar2base", {0, 0, 0});
  printf("rpy_lidar2base(deg): %.3f , %.3f, %.3f\n", rpy_lidar2base[0], rpy_lidar2base[1], rpy_lidar2base[2]);

  q_lidar2base.setRPY(DEG2RAD(rpy_lidar2base[0]), DEG2RAD(rpy_lidar2base[1]), DEG2RAD(rpy_lidar2base[2]));
  printf("q_lidar2base(x, y, z, w): %.3f , %.3f, %.3f, %.3f\n", q_lidar2base.x(), q_lidar2base.y(), q_lidar2base.z(),
         q_lidar2base.w());
}

void Converter::toPointCloud2(const int output_lidar_type, const rbk::protocol::Message_MapLogData3D& data,
                              sensor_msgs::PointCloud2& msg) {
  switch (output_lidar_type) {
    case 0:
      to_normal(data, msg);
      break;
    case 1:
      to_helios(data, msg);
      break;
    default:
      printf("output_lidar_type not supported!!!\n");
      exit(1);
  }
}

void Converter::toImu(const rbk::protocol::Message_IMU& data, sensor_msgs::Imu& imu) {
  double deg2rad = M_PI / 180.;

  imu.header.stamp = ros::Time().fromNSec(data.header().data_nsec());
  imu.header.frame_id = data.header().frame_id();
  imu.header.seq = data.header().seq();

  imu.orientation.x = data.qx();
  imu.orientation.y = data.qy();
  imu.orientation.z = data.qz();
  imu.orientation.w = data.qw();

  if (data.install_info().ssf() < 0.001) {
    imu.angular_velocity.x = (data.rot_x() + data.rot_off_x()) * deg2rad / 16.04;
    imu.angular_velocity.y = (data.rot_y() + data.rot_off_y()) * deg2rad / 16.04;
    imu.angular_velocity.z = (data.rot_z() + data.rot_off_z()) * deg2rad / 16.04;
  } else {
    imu.angular_velocity.x = (data.rot_x() + data.rot_off_x()) * deg2rad / data.install_info().ssf();
    imu.angular_velocity.y = (data.rot_y() + data.rot_off_y()) * deg2rad / data.install_info().ssf();
    imu.angular_velocity.z = (data.rot_z() + data.rot_off_z()) * deg2rad / data.install_info().ssf();
  }

  imu.linear_acceleration.x = data.acc_x();
  imu.linear_acceleration.y = data.acc_y();
  imu.linear_acceleration.z = data.acc_z();
}

void Converter::toOdom(const rbk::protocol::Message_MapOdo& data, nav_msgs::Odometry& odom) {
  odom.header.stamp = ros::Time().fromSec(data.timestamp());

  // 当前位姿
  Eigen::Affine3f pose = pcl::getTransformation(data.odo_x(), data.odo_y(), 0, 0, 0, data.odo_w());
  // 四元数
  Eigen::Quaternionf q(pose.rotation());

  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  odom.pose.pose.position.x = pose(0, 3);
  odom.pose.pose.position.y = pose(1, 3);
  odom.pose.pose.position.z = pose(2, 3);
}

static void from_helios(const sensor_msgs::PointCloud2::Ptr& msg, rbk::protocol::Message_MapLogData3D* data) {
  static const double time_offset_table[32] = {
      0.,           1.57 * 1e-6,  3.15 * 1e-6,  4.72 * 1e-6,  6.30 * 1e-6,  7.87 * 1e-6,  9.45 * 1e-6,  11.36 * 1e-6,
      13.26 * 1e-6, 15.17 * 1e-6, 17.08 * 1e-6, 18.99 * 1e-6, 20.56 * 1e-6, 22.14 * 1e-6, 23.71 * 1e-6, 25.29 * 1e-6,
      26.53 * 1e-6, 27.77 * 1e-6, 29.01 * 1e-6, 30.25 * 1e-6, 31.49 * 1e-6, 32.73 * 1e-6, 33.98 * 1e-6, 35.22 * 1e-6,
      36.46 * 1e-6, 37.70 * 1e-6, 38.94 * 1e-6, 40.18 * 1e-6, 41.42 * 1e-6, 42.67 * 1e-6, 43.91 * 1e-6, 45.15 * 1e-6};

  pcl::PointCloud<robosense_ros::Point> pc_data;
  pcl::fromROSMsg(*msg, pc_data);

  double first_points_time = pc_data.at(0).timestamp;
  // 设置为第一个点的时间
  data->set_timestamp(first_points_time);

  auto x = data->mutable_x();
  auto y = data->mutable_y();
  auto z = data->mutable_z();
  auto ring = data->mutable_ring();
  auto intensity = data->mutable_intensity();
  auto time_offset = data->mutable_timeoffset();

  x->Reserve(int(pc_data.size()));
  y->Reserve(int(pc_data.size()));
  z->Reserve(int(pc_data.size()));
  intensity->Reserve(int(pc_data.size()));
  ring->Reserve(int(pc_data.size()));
  time_offset->Reserve(int(pc_data.size()));

  // 检查点的时间顺序
  double last_point_time = 0;
  double last_point_time_corrected = 0;
  int fix_time_offset = 0;

  uint32_t t;
  for (size_t i = 0; i < pc_data.size(); i++) {
    if (pc_data.at(i).timestamp <= last_point_time) {
      fix_time_offset++;
      // 修正时间偏移
      pc_data.at(i).timestamp += time_offset_table[fix_time_offset];
      // printf("[%d - %d]: %.9f\n", i, fix_time_offset, pc_data.at(i).timestamp);
    } else {
      fix_time_offset = 0;
      last_point_time = pc_data.at(i).timestamp;
      // if (i) {
      //     printf("\ngap time(us): %.3f\n", (pc_data.at(i).timestamp - pc_data.at(i - 1).timestamp) * 1e6);
      // }
      // printf("[%d - %d]: %.9f\n", i, fix_time_offset, pc_data.at(i).timestamp);
    }

    if (pc_data.at(i).timestamp <= last_point_time_corrected) {
      printf("[%zu] points timeback!!last_point_time:%.9f, cur_point_time:%.9f\n", i, last_point_time_corrected,
             pc_data.at(i).timestamp);
      continue;
    } else {
      last_point_time_corrected = pc_data.at(i).timestamp;
    }

    // 判断一个就够了
    if (std::isinf(pc_data.at(i).x) || std::isnan(pc_data.at(i).x)) {
      continue;
    }

    x->AddAlreadyReserved(pc_data.at(i).x);
    y->AddAlreadyReserved(pc_data.at(i).y);
    z->AddAlreadyReserved(pc_data.at(i).z);
    intensity->AddAlreadyReserved(pc_data.at(i).intensity);
    ring->AddAlreadyReserved(pc_data.at(i).ring);
    t = static_cast<uint32_t>((pc_data.at(i).timestamp - first_points_time) * 1e9);
    time_offset->AddAlreadyReserved(t);
  }

  // printf("in size:%zu, out size: %d\n", pc_data.size(), x->size());
}

static void from_normal(const sensor_msgs::PointCloud2::Ptr& msg, rbk::protocol::Message_MapLogData3D* data) {
  pcl::PointCloud<PointXYZIRT> pc_data;
  pcl::fromROSMsg(*msg, pc_data);

  // 包时间
  double packet_time = msg->header.stamp.toSec();
  double first_points_time = packet_time + pc_data.at(0).time_offset;
  // 设置为第一个点的时间
  data->set_timestamp(first_points_time);

  auto x = data->mutable_x();
  auto y = data->mutable_y();
  auto z = data->mutable_z();
  auto ring = data->mutable_ring();
  auto intensity = data->mutable_intensity();
  auto time_offset = data->mutable_timeoffset();

  x->Reserve(int(pc_data.size()));
  y->Reserve(int(pc_data.size()));
  z->Reserve(int(pc_data.size()));
  intensity->Reserve(int(pc_data.size()));
  ring->Reserve(int(pc_data.size()));
  time_offset->Reserve(int(pc_data.size()));

  // 检查点的时间顺序
  double last_point_time = 0;
  // 当前点的时间
  double cur_point_time;

  uint32_t t;
  for (size_t i = 0; i < pc_data.size(); i++) {
    // 计算当前点的时间
    cur_point_time = packet_time + pc_data.at(i).time_offset;

    if (cur_point_time <= last_point_time) {
      printf("[%zu] points timeback!!last_point_time:%.9f, cur_point_time:%.9f\n", i, last_point_time, cur_point_time);
      continue;
    } else {
      last_point_time = cur_point_time;
    }

    // 判断一个就够了
    if (std::isinf(pc_data.at(i).x) || std::isnan(pc_data.at(i).x)) {
      continue;
    }

    x->AddAlreadyReserved(pc_data.at(i).x);
    y->AddAlreadyReserved(pc_data.at(i).y);
    z->AddAlreadyReserved(pc_data.at(i).z);
    intensity->AddAlreadyReserved(uint32_t(pc_data.at(i).intensity));
    ring->AddAlreadyReserved(pc_data.at(i).ring);
    t = static_cast<uint32_t>((cur_point_time - first_points_time) * 1e9);
    time_offset->AddAlreadyReserved(t);
  }

  // printf("in size:%zu, out size: %d\n", pc_data.size(), x->size());
}

void Converter::toMapLogData3d(const int input_lidar_type, const sensor_msgs::PointCloud2::Ptr& msg,
                               rbk::protocol::Message_MapLogData3D* data) {
  switch (input_lidar_type) {
    case 0:
      from_normal(msg, data);
      break;
    case 1:
      from_helios(msg, data);
      break;
    default:
      printf("input_lidar_type not supported!!!\n");
      exit(1);
  }
}

void Converter::toImu(const sensor_msgs::Imu::Ptr& imu, rbk::protocol::Message_IMU* data) {
  data->mutable_header()->set_frame_id(imu->header.frame_id);
  data->mutable_header()->set_seq(imu->header.seq);
  data->mutable_header()->set_data_nsec(imu->header.stamp.toNSec());

  tf::Transform cur_orientation{
      tf::Quaternion{imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w},
      tf::Vector3{0.0, 0.0, 0.0}};
  double roll, pitch, yaw;
  cur_orientation.getBasis().getRPY(roll, pitch, yaw);

  data->set_roll(roll);
  data->set_pitch(pitch);
  data->set_yaw(yaw);
  data->set_acc_x(imu->linear_acceleration.x);
  data->set_acc_y(imu->linear_acceleration.y);
  data->set_acc_z(imu->linear_acceleration.z);
  // change to deg/s
  data->set_rot_x(imu->angular_velocity.x * 57.29578);
  data->set_rot_y(imu->angular_velocity.y * 57.29578);
  data->set_rot_z(imu->angular_velocity.z * 57.29578);
  data->set_rot_off_x(0.);
  data->set_rot_off_y(0.);
  data->set_rot_off_z(0.);
  data->set_qx(imu->orientation.x);
  data->set_qy(imu->orientation.y);
  data->set_qz(imu->orientation.z);
  data->set_qw(imu->orientation.w);
}

}  // namespace seer