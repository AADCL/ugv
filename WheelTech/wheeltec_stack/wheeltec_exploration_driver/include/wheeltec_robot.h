
#ifndef __WHEELTEC_ROBOT_H_
#define __WHEELTEC_ROBOT_H_

#include "ros/ros.h"
#include <ros/message_event.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <string.h>
#include <string>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <serial/serial.h>
#include <fcntl.h>
#include <stdbool.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
using namespace std;

//Macro definition
//宏定义
#define SEND_DATA_CHECK   1          //Send data check flag bits //发送数据校验标志位
#define READ_DATA_CHECK   0          //Receive data to check flag bits //接收数据校验标志位
#define FRAME_HEADER      0X7B       //Frame head //帧头
#define FRAME_TAIL        0X7D       //Frame tail //帧尾
#define RECEIVE_DATA_SIZE 24         //The length of the data sent by the lower computer //下位机发送过来的数据的长度
#define SEND_DATA_SIZE    11         //The length of data sent by ROS to the lower machine //ROS向下位机发送的数据的长度
#define PI 				  3.1415926f //PI //圆周率

//Relative to the range set by the IMU gyroscope, the range is ±500°, corresponding data range is ±32768
//The gyroscope raw data is converted in radian (rad) units, 1/65.5/57.30=0.00026644
//与IMU陀螺仪设置的量程有关，量程±500°，对应数据范围±32768
//陀螺仪原始数据转换位弧度(rad)单位，1/65.5/57.30=0.00026644
#define GYROSCOPE_RATIO   0.00026644f
//Relates to the range set by the IMU accelerometer, range is ±2g, corresponding data range is ±32768
//Accelerometer original data conversion bit m/s^2 units, 32768/2g=32768/19.6=1671.84
//与IMU加速度计设置的量程有关，量程±2g，对应数据范围±32768
//加速度计原始数据转换位m/s^2单位，32768/2g=32768/19.6=1671.84
#define ACCEl_RATIO 	  1671.84f

extern sensor_msgs::Imu Mpu6050; //External variables, IMU topic data //外部变量，IMU话题数据

namespace wheeltec_driver_safety
{
constexpr const char* kDriverCmdVelTopic = "/wheeltec_driver/cmd_vel";
constexpr const char* kExpectedCmdVelCallerId = "/wheeltec_safety";
constexpr double kDefaultCmdVelTimeout = 0.20;
constexpr double kMinCmdVelTimeout = 0.05;
constexpr double kMaxCmdVelTimeout = 0.25;
constexpr double kDefaultCommandPublishRate = 20.0;
constexpr double kMinCommandPublishRate = 20.0;
constexpr double kMaxCommandPublishRate = 100.0;
constexpr double kMaxLinearX = 0.20;
constexpr double kMaxAngularZ = 0.40;
constexpr int kMinSerialTimeoutMs = 1;
constexpr int kMaxSerialTimeoutMs = 10;
constexpr int kMinStopRepeats = 3;
constexpr int kMaxStopRepeats = 20;
constexpr double kMaxStopRepeatInterval = 0.10;

inline double ClampFiniteParameter(double value, double fallback,
                                   double minimum, double maximum)
{
	if (!std::isfinite(value))
		return fallback;
	return std::max(minimum, std::min(maximum, value));
}

inline int ClampIntegerParameter(int value, int minimum, int maximum)
{
	return std::max(minimum, std::min(maximum, value));
}

inline bool IsExpectedCmdVelPublisher(const ros::M_string& connection_header)
{
	const ros::M_string::const_iterator caller =
	    connection_header.find("callerid");
	return caller != connection_header.end() &&
	       caller->second == kExpectedCmdVelCallerId;
}

inline bool IsInvariantDriverTopic(const std::string& resolved_topic)
{
	return resolved_topic == kDriverCmdVelTopic;
}

class CmdVelSourceGuard
{
	public:
		bool Accept(const ros::M_string* connection_header)
		{
			std::lock_guard<std::mutex> lock(mutex_);
			if (fault_latched_)
				return false;
			if (connection_header == nullptr ||
			    !IsExpectedCmdVelPublisher(*connection_header))
			{
				fault_latched_ = true;
				return false;
			}
			return true;
		}

		bool IsFaultLatched() const
		{
			std::lock_guard<std::mutex> lock(mutex_);
			return fault_latched_;
		}

	private:
		mutable std::mutex mutex_;
		bool fault_latched_ = false;
};
}  // namespace wheeltec_driver_safety

inline geometry_msgs::Twist LimitCmdVel(const geometry_msgs::Twist& command,
                                        double max_linear_x,
                                        double max_linear_y,
                                        double max_angular_z)
{
	const auto limit = [](double value, double maximum) {
		const double bound = std::max(0.0, maximum);
		return std::max(-bound, std::min(bound, value));
	};
	geometry_msgs::Twist limited;
	limited.linear.x = limit(command.linear.x, max_linear_x);
	limited.linear.y = limit(command.linear.y, max_linear_y);
	limited.angular.z = limit(command.angular.z, max_angular_z);
	return limited;
}

// Thread-safe command lease. The chassis stays locked at zero until the first
// command arrives, and automatically returns to zero when the lease expires.
class CmdVelWatchdog
{
	public:
		using Clock = std::chrono::steady_clock;
		using TimePoint = Clock::time_point;

		void SetTimeout(double timeout_seconds)
		{
			std::lock_guard<std::mutex> lock(mutex_);
			timeout_ = std::chrono::duration<double>(timeout_seconds);
		}

		void Update(const geometry_msgs::Twist& command,
		            const TimePoint& now = Clock::now())
		{
			std::lock_guard<std::mutex> lock(mutex_);
			command_ = command;
			last_command_time_ = now;
			has_command_ = true;
		}

		geometry_msgs::Twist CommandFor(
		    const TimePoint& now = Clock::now(), bool* fresh = nullptr) const
		{
			std::lock_guard<std::mutex> lock(mutex_);
			const bool command_is_fresh =
			    has_command_ && now >= last_command_time_ &&
			    std::chrono::duration<double>(now - last_command_time_) <= timeout_;
			if (fresh != nullptr)
				*fresh = command_is_fresh;
			return command_is_fresh ? command_ : geometry_msgs::Twist();
		}

		void Lock()
		{
			std::lock_guard<std::mutex> lock(mutex_);
			command_ = geometry_msgs::Twist();
			has_command_ = false;
		}

	private:
		mutable std::mutex mutex_;
		geometry_msgs::Twist command_;
		TimePoint last_command_time_;
		std::chrono::duration<double> timeout_{
		    wheeltec_driver_safety::kDefaultCmdVelTimeout};
		bool has_command_ = false;
};

//Covariance matrix for speedometer topic data for robt_pose_ekf feature pack
//协方差矩阵，用于里程计话题数据，用于robt_pose_ekf功能包
const double odom_pose_covariance[36]   = {1e-3,    0,    0,   0,   0,    0,
										      0, 1e-3,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0,  1e3 };

const double odom_pose_covariance2[36]  = {1e-9,    0,    0,   0,   0,    0,
										      0, 1e-3, 1e-9,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0, 1e-9 };

const double odom_twist_covariance[36]  = {1e-3,    0,    0,   0,   0,    0,
										      0, 1e-3,    0,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0,  1e3 };

const double odom_twist_covariance2[36] = {1e-9,    0,    0,   0,   0,    0,
										      0, 1e-3, 1e-9,   0,   0,    0,
										      0,    0,  1e6,   0,   0,    0,
										      0,    0,    0, 1e6,   0,    0,
										      0,    0,    0,   0, 1e6,    0,
										      0,    0,    0,   0,   0, 1e-9} ;

//Data structure for speed and position
//速度、位置数据结构体
typedef struct __Vel_Pos_Data_
{
	float X;
	float Y;
	float Z;
}Vel_Pos_Data;

//IMU data structure
//IMU数据结构体
typedef struct __MPU6050_DATA_
{
	short accele_x_data;
	short accele_y_data;
	short accele_z_data;
    short gyros_x_data;
	short gyros_y_data;
	short gyros_z_data;

}MPU6050_DATA;

//The structure of the ROS to send data to the down machine
//ROS向下位机发送数据的结构体
typedef struct _SEND_DATA_
{
	    uint8_t tx[SEND_DATA_SIZE];
		float X_speed;
		float Y_speed;
		float Z_speed;
		unsigned char Frame_Tail;
}SEND_DATA;

//The structure in which the lower computer sends data to the ROS
//下位机向ROS发送数据的结构体
typedef struct _RECEIVE_DATA_
{
	    uint8_t rx[RECEIVE_DATA_SIZE];
	    uint8_t Flag_Stop;
		unsigned char Frame_Header;
		float X_speed;
		float Y_speed;
		float Z_speed;
		float Power_Voltage;
		unsigned char Frame_Tail;
}RECEIVE_DATA;

//The robot chassis class uses constructors to initialize data, publish topics, etc
//机器人底盘类，使用构造函数初始化数据和发布话题等
class turn_on_robot
{
	public:
		turn_on_robot();  //Constructor //构造函数
		~turn_on_robot(); //Destructor //析构函数
		void Control();   //Loop control code //循环控制代码
		serial::Serial Stm32_Serial; //Declare a serial object //声明串口对象
	private:
		ros::NodeHandle n;           //Create a ROS node handle //创建ROS节点句柄
		ros::Time _Now, _Last_Time;  //Time dependent, used for integration to find displacement (mileage) //时间相关，用于积分求位移(里程)
		float Sampling_Time;         //Sampling time, used for integration to find displacement (mileage) //采样时间，用于积分求位移(里程)

		ros::Subscriber Cmd_Vel_Sub; //Initialize the topic subscriber //初始化话题订阅者
		//The speed topic subscribes to the callback function
		//速度话题订阅回调函数
		void Cmd_Vel_Callback(
		    const ros::MessageEvent<geometry_msgs::Twist const>& event);
		void Command_Timer_Callback(const ros::WallTimerEvent& event);
		void Send_Velocity_Command(const geometry_msgs::Twist& command);
		bool Write_Velocity_Command_Locked(const geometry_msgs::Twist& command);
		void Send_Stop_Burst(int repeat_count);
		size_t Read_Serial(uint8_t* buffer, size_t size);
		void Begin_Shutdown();

		ros::Publisher odom_publisher, imu_publisher, voltage_publisher; //Initialize the topic publisher //初始化话题发布者
		void Publish_Odom();      //Pub the speedometer topic //发布里程计话题
		void Publish_ImuSensor(); //Pub the IMU sensor topic //发布IMU传感器话题
		void Publish_Voltage();   //Pub the power supply voltage topic //发布电源电压话题

        //从串口(ttyUSB)读取运动底盘速度、IMU、电源电压数据
        //Read motion chassis speed, IMU, power supply voltage data from serial port (ttyUSB)
        bool Get_Sensor_Data();
		bool Get_Sensor_Data_New();
        unsigned char Check_Sum(unsigned char Count_Number,unsigned char mode); //BBC check function //BBC校验函数
        short IMU_Trans(uint8_t Data_High,uint8_t Data_Low);  //IMU data conversion read //IMU数据转化读取
		float Odom_Trans(uint8_t Data_High,uint8_t Data_Low); //Odometer data is converted to read //里程计数据转化读取

        string usart_port_name, robot_frame_id, gyro_frame_id, odom_frame_id; //Define the related variables //定义相关变量
        int serial_baud_rate;      //Serial communication baud rate //串口通信波特率
        RECEIVE_DATA Receive_Data; //The serial port receives the data structure //串口接收数据结构体
        SEND_DATA Send_Data;       //The serial port sends the data structure //串口发送数据结构体

        Vel_Pos_Data Robot_Pos;    //The position of the robot //机器人的位置
        Vel_Pos_Data Robot_Vel;    //The speed of the robot //机器人的速度
        MPU6050_DATA Mpu6050_Data; //IMU data //IMU数据
        float Power_voltage;       //Power supply voltage //电源电压

		ros::WallTimer Command_Timer;
		CmdVelWatchdog Cmd_Watchdog;
		wheeltec_driver_safety::CmdVelSourceGuard Cmd_Source_Guard;
		std::mutex Serial_Mutex;
		std::atomic<bool> Initialization_Succeeded{false};
		std::atomic<bool> Shutting_Down{false};
		std::atomic<bool> Last_Command_Was_Fresh{false};
		// Readers yield while any command writer is waiting. A counter preserves
		// priority if a timer write and a shutdown write overlap.
		std::atomic<unsigned int> Pending_Serial_Writes{0};
		double cmd_vel_timeout = wheeltec_driver_safety::kDefaultCmdVelTimeout;
		double command_publish_rate =
		    wheeltec_driver_safety::kDefaultCommandPublishRate;
		double stop_repeat_interval = 0.02;
		double max_linear_x = wheeltec_driver_safety::kMaxLinearX;
		double max_linear_y = 0.0;
		double max_angular_z = wheeltec_driver_safety::kMaxAngularZ;
		int serial_read_timeout_ms =
		    wheeltec_driver_safety::kMaxSerialTimeoutMs;
		int startup_stop_repeats = 5;
		int shutdown_stop_repeats = 5;
};
#endif
