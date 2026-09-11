# 2026-09-11 实车室内试用与相机运行库修复

用户授权使用此前六场景画廊`prior_robust_final_20260911/loose.yaml`，备份原配置后试用；不是后来墙角批次结果。安装初值未覆盖，试用入口为`~/r3live_ws/calibration/trials/indoor_20260911_01/start.sh`。R³LIVE、FAST-LIO、NDT和原导航参数没有合并。

## 故障证据与修复

前两次真实启动在相机nodelet收到图像压缩订阅后SIGSEGV，LIO已有输出但相机位姿超时。apport core中调用栈为compressed_image_transport → overlay cv_bridge → OpenCV4.5 cvtColor → OpenCV4.2 _OutputArray::create。旧RealSense库本身也直接链接4.2与4.5。

在r3live_ws克隆固定版本RealSense2.3.2与image_transport_plugins，显式OpenCV依赖补丁，随cv_bridge在相同overlay串行重编。未修改旧realsense_ws、系统库或原导航工作空间。入口取消前置旧相机搜索路径，增加全部相关库ldd检查；标定入口优先使用新overlay。

`catkin_make -j1 -DCMAKE_BUILD_TYPE=Release -DR3LIVE_BUILD_MESHING=OFF`成功。相机、cv_bridge、compressed、compressedDepth、theora与mapping共6项运行库检查只出现OpenCV4.5。真实相机进程`/proc/188326/maps`确认只有4.5.4，并从r3live_ws加载相机及插件。

## 实际外参与输出

成功会话：`sessions/20260911_162346_a6i2hm76`。`runtime.yaml`对`expected_runtime_extrinsic.yaml`最大元素误差为0。R³LIVE实际使用的`T_imu_camera`约为：

```text
 0.01418314  0.72873359  0.68465041  0.06320351
-0.99892842 -0.01984173  0.04181298 -0.02099246
 0.04405517 -0.68450979  0.72767128 -0.03161235
 0           0           0           1
```

16:29:18～16:29:48的30秒观测：LIO301帧约10.02Hz，相机位姿446帧约14.88Hz，原图449帧约14.93Hz，IMU6011帧约200.07Hz。所有观测时间戳单调，未见超过2秒间断或非有限位姿。LIO相对首帧最大变化约4.49cm/0.86°，相机约4.28cm/0.77°；未取得该窗口全程静止的独立确认，不能称为定位精度。16:28:18～16:28:28较大位移及转角经用户确认属于人工移车转弯，不应作为静止漂移。

16:30:57～59日志显示tracked108～117，geometric=1、photometric=1，证明视觉更新分支确实运行。约7分钟时相机RSS65MiB、mapping338MiB；不代表长时资源验收。

早期两次观察报告中raw图像接收年龄为约-1.9秒，保留原报告、不宣称已定位根因。随后独立订阅原图、压缩图、CameraInfo、Metadata，确认同帧时间一致且global_time域接收延迟约40～60ms；后续30秒图像中位接收年龄46ms、IMU0.43ms。未改写传感器时间戳或外参来消除异常。新观察脚本单独报告sensor_clock_sanity，超出-0.1～0.5秒返回失败；这只是时钟粗检，不是时间偏移标定。

## 停止与数据

用户告知没电、移车充电后，以SIGINT停止本次录包及试用入口，确认估计器、相机和录包进程退出。3个分包均已正常关闭、rosbag info可读取：

| 文件 | 设备时段 | 消息数 |
|---|---|---:|
| bags/indoor_retry2_0.bag | 16:29:12.72～16:30:13.53 | 17010 |
| bags/indoor_retry2_1.bag | 16:30:13.54～16:31:14.29 | 17041 |
| bags/indoor_retry2_2.bag | 16:31:14.29～16:31:16.04 | 494 |

相对路径均位于`~/r3live_ws/calibration/trials/indoor_20260911_01/`，共约2.1GiB。保留Livox原始点云/IMU、raw彩色图/内参/元数据、tf_static、两路位姿和rosout；16:28的那次已确认运动早于录包，不在这些bag里。

本地`D:/设备文档/scout_calibration_review_20260911/`保留配置备份归档及会话诊断。大bag保留设备，不上传GitHub。当前通过真实启动与输出检查；精确外参、时间标定、运动精度、返回起点误差及长管廊性能仍待后续测试。充电后用同一试用start.sh新建会话，观察和bag文件使用新名字。
