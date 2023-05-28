该工具提供如下功能：

- [x] 将rawmap转为rosbag；
- [x] 将rosbag转为rawmap；
- [ ] 实时接收机器人传感器数据转为ros标准消息发出；

# 启动

## rawmap转为rosbag

~~*注意目前激光数据转换之后每个点的时间不是偏移时间，而是真实值*~~
支持将rawmap转为多种激光类型：
- normal - 每个点的时间由offset构成;
- helios - 每个点的时候是真实的时间，但是helios的时间需要额外处理;

详情见./launch/convert_to_rosbag.launch文件。

```bash
# 方法一
# 修改./launch/convert_to_rosbag.launch文件中的input_file_path与output_rosbag_path后，运行
roslaunch rosmsg_converter convert_to_rosbag.launch

# 方法二
# 地图文件转为rosbag
rosrun rosmsg_converter rosmsg_converter 地图文件路径 保存rosbag路径
# 例如
rosrun rosmsg_converter rosmsg_converter /home/anson/share/data/Robokit_2021-06-18_15-00-24.rawmap /home/anson/rosbag/converted.bag

```

转换后可以在/info_converted话题中看到外参信息。

```bash
$ rostopic echo /info_converted
data: "t_lidar2base(x, y, z): 0.055, 0.022, -0.030, R_lidar2base(rpy, deg): 0.000 , 0.000,\
  \ 0.000, t_imu2base(x, y, z): 0.000, 0.000, 0.000, R_imu2base(rpy, deg): 0.000 ,\
  \ -0.000, 0.000"

```

## rosbag转为rawmap

rawmap激光数据格式：

```protobuf
message Message_MapLogData3D {
    double timestamp = 1;
    repeated float x = 2;
    repeated float y = 3;
    repeated float z = 4;
    repeated uint32 intensity = 5;
    repeated uint32 timeoffset = 6;
    repeated uint32 ring = 7;
}
```

转换后每帧激光的时间为第一个点的时间，即timestamp的值，timeoffset是每个激光点与第一激光的差值，这里要求第一个点的时间必须最早，后续点在时间上递增。转换后激光数据中没有无效值。

进行转换时还需要设置imu与lidar的外参，输入激光的类型（input_lidar_type）。详情见./launch/convert_to_rawmap.launch文件。设置partial_convert_ratio参数可以转换rosbag的一部分。

```bash
# 修改./launch/convert_to_rawmap.launch文件中的input_file_path与output_rawmao_path后，运行
roslaunch rosmsg_converter convert_to_rawmap.launch
```

# todo

- [x] 支持不同的激光雷达数据；

#todo git best

#todo   
#测试了主分支和dev分支