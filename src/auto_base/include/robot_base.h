#ifndef ROBOMASTER_ROBOT_H
#define ROBOMASTER_ROBOT_H

#include "serial_device.h"
#include "protocol.h"
#include "crc.h"

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <thread>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>


double x = 0.0;
double y = 0.0;
double th = 0.0;
double yaw = 0;
geometry_msgs::msg::Quaternion Imu_quat;

rclcpp::Time current_time, last_time;

namespace robomaster {
class Robot : public rclcpp::Node {
 public:
  Robot(std::string device_path = "/dev/robomaster");
  ~Robot();
  void ChassisCtrlCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

 private:
  bool ROSInit();
  bool CommInit();
  void RecvThread();
  void SearchFrameSOF(uint8_t *frame, uint16_t total_len);
  uint16_t ReceiveDataSolve(uint8_t *frame);
  uint16_t SenderPackSolve(uint8_t *data, uint16_t data_length, uint16_t cmd_id, uint8_t *send_buf);

 private:
  //! VCOM Data Receiving Thread (Tips: VCOM Sending part is in Each ROS Data Callback)
  // std::thread recv_thread_;
  //! Device Information and Buffer Allocation
  std::string device_path_;
  std::shared_ptr<SerialDevice> device_ptr_;
 
  std::shared_ptr<std::thread> recv_thread_;  // 改为智能指针
  rclcpp::Clock::SharedPtr clock ;

  std::unique_ptr<uint8_t[]> recv_buff_;
  std::unique_ptr<uint8_t[]> send_buff_;
  const unsigned int BUFF_LENGTH = 512;
  //! Frame Information
  frame_header_struct_t frame_receive_header_;
  frame_header_struct_t frame_send_header_;
  /** @brief specific protocol data are defined here
   *         xxxx_info_t is defined in protocol.h
   */
  //! Receive from VCOM 
  
  chassis_odom_info_t chassis_odom_info_; //! ros chassis odometry 
  geometry_msgs::msg::TransformStamped odom_tf_;//! ros chassis odometry tf
  nav_msgs::msg::Odometry odom_;//! ros odometry message
  //! Send to VCOM
  chassis_ctrl_info_t chassis_ctrl_info_;


  /** @brief ROS data corresponding to specific protocol data are defined here
   *         You can use ROS provided message data type or create your own one
   *         More information please refer to
   *               http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
   */
  /** @brief ROS Subscription and Publication
   */
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr chassis_ctrl_sub_; //! ros subscriber for chassis velocity control
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr chassis_odom_pub_; //! ros publisher for odometry information
  tf2_ros::TransformBroadcaster tf_broadcaster_;//! ros chassis odometry tf broadcaster
};
}

#endif //ROBOMASTER_ROBOT_H
