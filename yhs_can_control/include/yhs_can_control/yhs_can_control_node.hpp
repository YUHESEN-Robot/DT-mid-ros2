#ifndef __YHS_CANCONTROL_NODE_H__
#define __YHS_CANCONTROL_NODE_H__

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "nav_msgs/msg/odometry.hpp"

#include "yhs_can_interfaces/msg/io_cmd.hpp"
#include "yhs_can_interfaces/msg/front_free_ctrl_cmd.hpp"
#include "yhs_can_interfaces/msg/rear_free_ctrl_cmd.hpp"
#include "yhs_can_interfaces/msg/ctrl_cmd.hpp"
#include "yhs_can_interfaces/msg/chassis_info_fb.hpp"

#define READ_PARAM(TYPE, NAME, VAR, VALUE)  \
  VAR = VALUE;                              \
  node->declare_parameter<TYPE>(NAME, VAR); \
  node->get_parameter(NAME, VAR);

namespace yhs
{
  class CanControl
  {

  public:
    CanControl(rclcpp::Node::SharedPtr);
    ~CanControl();

    bool run();
    void stop();

  private:
    rclcpp::Node::SharedPtr node_;

    std::string if_name_;
    int can_socket_;
    std::thread thread_;
    std::mutex cmd_mutex_;
    std::vector<int64_t> ultrasonic_number_;

    yhs_can_interfaces::msg::IoCmd current_io_cmd_;
    yhs_can_interfaces::msg::CtrlCmd current_ctrl_cmd_;

    rclcpp::Subscription<yhs_can_interfaces::msg::IoCmd>::SharedPtr io_cmd_subscriber_;
    rclcpp::Subscription<yhs_can_interfaces::msg::CtrlCmd>::SharedPtr ctrl_cmd_subscriber_;
    rclcpp::Subscription<yhs_can_interfaces::msg::FrontFreeCtrlCmd>::SharedPtr front_free_ctrl_cmd_subscriber_;
    rclcpp::Subscription<yhs_can_interfaces::msg::RearFreeCtrlCmd>::SharedPtr rear_free_ctrl_cmd_subscriber_;    

    rclcpp::Publisher<yhs_can_interfaces::msg::ChassisInfoFb>::SharedPtr chassis_info_fb_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<yhs_can_interfaces::msg::OdoFb>::SharedPtr odo_fb_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback();

    void send_ctrl_cmd();
   
    void send_io_cmd();

    void io_cmd_callback(const yhs_can_interfaces::msg::IoCmd::SharedPtr io_cmd_msg);

    void ctrl_cmd_callback(const yhs_can_interfaces::msg::CtrlCmd::SharedPtr ctrl_cmd_msg);

    void front_free_ctrl_cmd_callback(const yhs_can_interfaces::msg::FrontFreeCtrlCmd::SharedPtr front_free_ctrl_cmd_msg);

    void rear_free_ctrl_cmd_callback(const yhs_can_interfaces::msg::RearFreeCtrlCmd::SharedPtr rear_free_ctrl_cmd_msg);

    bool wait_for_can_frame();

    void can_data_recv_callback();

    void publish_odom(double linear_vel, double angular_vel);
  };

}

#endif
