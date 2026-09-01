# Orbbec Gemini 336L 相机说明

## 已部署版本

- 设备：Orbbec Gemini 336L，USB ID `2bc5:0807`
- 驱动：OrbbecSDK_ROS1 `v2-main`，提交 `a2838b3`
- ROS Wrapper / SDK：`2.9.3`
- NX 路径：`~/livox_fastlio/src/OrbbecSDK_ROS1`
- 相机固件实测：`1.4.60`
- 连接实测：USB 3.2

官方驱动在该 NX 上需要 `third_party_patches/orbbec_ros1_aarch64_link_order.patch`，同时 OpenCV 开发头文件与 Ubuntu 20.04/ROS Noetic 的 4.2.0 运行库必须一致。

## 一键启动

```bash
cd ~/livox_fastlio
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch wheeltec_system_bringup wheeltec_orbbec336l.launch
```

默认启动 RGB、深度和无颜色点云，关闭红外与相机 IMU。实测输出为：

| 话题 | 类型 | 帧 | 实测频率/分辨率 |
| --- | --- | --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/Image` | `camera_color_optical_frame` | 30 Hz，1280×720 MJPG |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | `camera_color_optical_frame` | 随 RGB 输出 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | `camera_depth_optical_frame` | 约 30 Hz，848×480 Y16 |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | `camera_depth_optical_frame` | 随深度输出 |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | `camera_depth_optical_frame` | 约 30 Hz |

驱动按订阅者需求启动数据流；没有订阅时不持续传输图像是正常现象。

## 安装 TF

启动文件发布：

```text
base_link -> camera_link -> camera_*_optical_frame
```

其中 `camera_link -> camera_*_optical_frame` 来自相机标定，`base_link -> camera_link` 由本项目发布。正式外参为前 `0.16 m`、左 `0 m`、上 `0.08 m`，相机与车体同姿态，因此 `roll=pitch=yaw=0`。

```bash
roslaunch wheeltec_system_bringup wheeltec_orbbec336l.launch \
  camera_x:=0.16 camera_y:=0.0 camera_z:=0.08 \
  camera_roll:=R camera_pitch:=P camera_yaw:=W
```

单位为米和弧度，车体坐标约定为前 `+X`、左 `+Y`、上 `+Z`。上述平移已写入 `wheeltec_orbbec336l.launch` 默认参数。

若只测试驱动、不希望接入整车 TF：

```bash
roslaunch wheeltec_system_bringup wheeltec_orbbec336l.launch publish_mount_tf:=false
```

## 快速检查

```bash
lsusb | grep 2bc5:0807
rostopic hz /camera/color/image_raw
rostopic hz /camera/depth/image_raw
rostopic hz /camera/depth/points
rosrun tf tf_echo base_link camera_link
```
