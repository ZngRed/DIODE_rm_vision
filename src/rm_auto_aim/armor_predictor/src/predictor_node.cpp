#include "armor_predictor/predictor_node.hpp"

// STD
#include <memory>
#include <vector>

namespace rm_auto_aim
{
    // 从坐标轴正向看向原点，逆时针方向为正
ArmorPredictorNode::ArmorPredictorNode(const rclcpp::NodeOptions & options)
: Node("armor_predictor", options)
{
    RCLCPP_WARN(this->get_logger(), "Starting PredictorNode!");

    target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
      "/tracker/target", rclcpp::SensorDataQoS(),
      std::bind(&ArmorPredictorNode::targetCallback, this, std::placeholders::_1));

    state_sub_ = this->create_subscription<auto_aim_interfaces::msg::State>(
      "/state", rclcpp::SensorDataQoS(),
      std::bind(&ArmorPredictorNode::stateCallback, this, std::placeholders::_1));

    // Publisher
    aiming_pub_ = this->create_publisher<auto_aim_interfaces::msg::Aiming>(
        "/predictor/aiming", rclcpp::SensorDataQoS());

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);

    aiming_point_.header.frame_id = "odom";
    aiming_point_.ns = "aiming_point";
    aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
    aiming_point_.action = visualization_msgs::msg::Marker::ADD;
    aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
    aiming_point_.color.r = 1.0;
    aiming_point_.color.g = 1.0;
    aiming_point_.color.b = 1.0;
    aiming_point_.color.a = 1.0;
    aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);
}

void ArmorPredictorNode::targetCallback(const auto_aim_interfaces::msg::Target::SharedPtr target_msg)
{
  float aim_x = 0, aim_y = 0, aim_z = 0; // aim point 落点，传回用于可视化
  float tgt_pitch = 0; //控制量 pitch绝对角度 弧度
  float tgt_yaw = 0;   //控制量 yaw绝对角度 弧度

  predictor_.st.current_v = state_v;
  predictor_.st.current_pitch = state_pitch;
  predictor_.st.current_yaw = state_yaw;
  try {  
    switch(std::stoi(target_msg->id)){
      case 0:
        predictor_.st.armor_id = ARMOR_OUTPOST;
        break;
      case 1:
        predictor_.st.armor_id = ARMOR_HERO;
        break;
      case 2:
        predictor_.st.armor_id = ARMOR_ENGINEER;
        break;
      case 3:
        predictor_.st.armor_id = ARMOR_INFANTRY3;
        break;
      case 4:
        predictor_.st.armor_id = ARMOR_INFANTRY4;
        break;
      case 5:
        predictor_.st.armor_id = ARMOR_INFANTRY5;
        break;
      case 6:
        predictor_.st.armor_id = ARMOR_GUARD;
        break;
      case 7:
        predictor_.st.armor_id = ARMOR_BASE;
        break;
    }
  } catch (const std::invalid_argument& e) {  
      std::cerr << "Invalid argument: " << e.what() << std::endl;  
  } catch (const std::out_of_range& e) {  
      std::cerr << "Out of range: " << e.what() << std::endl;  
  }  
  switch(target_msg->armors_num){
    case 2:
      predictor_.st.armor_num = ARMOR_NUM_BALANCE;
      break;
    case 3:
      predictor_.st.armor_num = ARMOR_NUM_OUTPOST;
      break;
    case 4:
      predictor_.st.armor_num = ARMOR_NUM_NORMAL;
      break;
  }
  predictor_.st.xw = target_msg->position.x;
  predictor_.st.yw = target_msg->position.y;
  predictor_.st.zw = target_msg->position.z;
  predictor_.st.vxw = target_msg->velocity.x;
  predictor_.st.vyw = target_msg->velocity.y;
  predictor_.st.vzw = target_msg->velocity.z;
  predictor_.st.tar_yaw = target_msg->yaw;
  predictor_.st.v_yaw = target_msg->v_yaw;
  predictor_.st.r1 = target_msg->radius_1;
  predictor_.st.r2 = target_msg->radius_2;
  predictor_.st.dz = target_msg->dz;

  predictor_.autoSolveTrajectory(&tgt_pitch, &tgt_yaw, &aim_x, &aim_y, &aim_z);

  // printf("main pitch:%f° yaw:%f° ", delta_pitch * 180 / PI, delta_yaw * 180 / PI);
  // printf("\npitch:%frad yaw:%frad aim_x:%f aim_y:%f aim_z:%f", delta_pitch, delta_yaw, aim_x, aim_y, aim_z);

  // Init message
  auto_aim_interfaces::msg::Aiming aiming_msg;

  rclcpp::Time time = target_msg->header.stamp;
  aiming_msg.header.stamp = time;
  aiming_msg.header.frame_id = target_msg->header.frame_id;
  aiming_msg.tracking = target_msg->tracking;
  aiming_msg.id = target_msg->id;
  aiming_msg.armors_num = target_msg->armors_num;
  aiming_msg.fire = 0;
  aiming_msg.position.x = target_msg->position.x;
  aiming_msg.position.y = target_msg->position.y;
  aiming_msg.position.z = target_msg->position.z;
  aiming_msg.pitch = tgt_pitch * 180 / PI;
  aiming_msg.yaw = tgt_yaw * 180 / PI;

  if(target_msg->armors_num == 0){
    aiming_msg.pitch = 0;
    aiming_msg.yaw = 0;
  }

  // RCLCPP_ERROR(this->get_logger(), "main p: %f y: %f | state p: %f y: %f", delta_pitch, delta_yaw, state_pitch, state_yaw);

  RCLCPP_WARN(this->get_logger(), "delta pitch:%f° yaw:%f° ", 
      (-1) * (tgt_pitch - state_pitch) * 180 / PI, (-1) * (tgt_yaw - state_yaw) * 180 / PI);

  RCLCPP_WARN(this->get_logger(), "armors_num %d", target_msg->armors_num);

  aiming_pub_->publish(aiming_msg);

  if (abs(aim_x) > 0.01) {
    aiming_point_.header.stamp = this->now();
    aiming_point_.pose.position.x = aim_x;
    aiming_point_.pose.position.y = aim_y;
    aiming_point_.pose.position.z = aim_z;
    marker_pub_->publish(aiming_point_);
  }
}

void ArmorPredictorNode::stateCallback(const auto_aim_interfaces::msg::State::SharedPtr state_msg)
{
  state_v = state_msg->v;
  state_pitch = state_msg->pitch;
  state_yaw = state_msg->yaw;
}

// void ArmorPredictorNode::publishAiming(const auto_aim_interfaces::msg::Target & target_msg)
// {
// }

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorPredictorNode)
