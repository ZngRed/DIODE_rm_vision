#ifndef ARMOR_PREDICTOR__PREDICTOR_NODE_HPP_
#define ARMOR_PREDICTOR__PREDICTOR_NODE_HPP_

// ROS
// #include <message_filters/subscriber.h>
#include <rclcpp/subscription.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_predictor/predictor.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/aiming.hpp"
#include "auto_aim_interfaces/msg/state.hpp"

namespace rm_auto_aim
{
class ArmorPredictorNode : public rclcpp::Node
{
public:
  explicit ArmorPredictorNode(const rclcpp::NodeOptions & options);

  Predictor predictor_;

  // Aimimg point receiving from serial port for visualization
  visualization_msgs::msg::Marker aiming_point_;

  float state_v = 0;
  float state_pitch = 0;
  float state_yaw = 0;
  int tar_count = 0;

private:
  void targetCallback(const auto_aim_interfaces::msg::Target::SharedPtr target_msg);
  void stateCallback(const auto_aim_interfaces::msg::State::SharedPtr state_msg);

//   void publishAiming(const auto_aim_interfaces::msg::Target & target_msg);

  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::State>::SharedPtr state_sub_;

  // Publisher
  rclcpp::Publisher<auto_aim_interfaces::msg::Aiming>::SharedPtr aiming_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PREDICTOR__PREDICTOR_NODE_HPP_
