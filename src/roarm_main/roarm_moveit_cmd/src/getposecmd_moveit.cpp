#include <chrono>
#include <thread>
#include <iomanip>
#include <sstream> 
#include <cstdlib> 
#include <string>
#include <iostream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "roarm_msgs/srv/get_pose_cmd.hpp" 
geometry_msgs::msg::Pose hand_pose;
std::vector<double> hand_pose_rpy;

std::string get_roarm_model() {
    const char* env_val = std::getenv("ROARM_MODEL");
    if (env_val == nullptr) {
        std::cerr << "no ROARM_MODEL!" << std::endl;
        return "";  
    }
    return std::string(env_val); 
}

template<typename T>
float format_to_6decimal(T value) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(6) << value;
    return std::stof(ss.str()); 
}

void handle_service(const std::shared_ptr<roarm_msgs::srv::GetPoseCmd::Request> request,
                    std::shared_ptr<roarm_msgs::srv::GetPoseCmd::Response> response)
{
  std::string model = get_roarm_model();
  response->x = format_to_6decimal(hand_pose.position.x);
  response->y = format_to_6decimal(hand_pose.position.y);
  if (model == "roarm_m2") {
    response->z = format_to_6decimal(hand_pose.position.z-0.1338);
    response->roll = 0.0;
    response->pitch = 0.0;
    response->yaw = 0.0;
  } else if (model == "roarm_m3") {
    response->z = format_to_6decimal(hand_pose.position.z-0.1221);
    response->roll = format_to_6decimal(hand_pose_rpy[0]);
    response->pitch = format_to_6decimal(hand_pose_rpy[1]);
    response->yaw = format_to_6decimal(hand_pose_rpy[2]);
  } 
}

int main(int argc, char** argv)
{
  using namespace std::chrono_literals;
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("get_pose_cmd_node", node_options);
  auto logger = node->get_logger();
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
  publisher_ = node->create_publisher<geometry_msgs::msg::Pose>("hand_pose", 1);
  auto server = node->create_service<roarm_msgs::srv::GetPoseCmd>("get_pose_cmd", &handle_service);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });
  
  moveit::planning_interface::MoveGroupInterface move_group_interface(node, "hand");
  geometry_msgs::msg::PoseStamped current_pose;

  while (rclcpp::ok())
  {
    current_pose = move_group_interface.getCurrentPose();
    hand_pose = current_pose.pose;
    hand_pose_rpy = move_group_interface.getCurrentRPY("hand_tcp");
    
    publisher_->publish(hand_pose);
    std::this_thread::sleep_for(1s);
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
