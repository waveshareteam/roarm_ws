#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include "roarm_msgs/srv/get_pose_cmd.hpp"
#include <iostream>
#include <cmath>
#include <iomanip>
#include <sstream> 
#include <cstdlib> 
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


class RobotPoseSubscription : public rclcpp::Node
{
public:
    RobotPoseSubscription() : Node("robot_pose_subscription")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("hand_pose", 1);
    }

    void update_end_effector_pose() 
    {
        try
        {
            geometry_msgs::msg::TransformStamped transformStamped;

            transformStamped = tf_buffer_->lookupTransform(base_frame, end_effector_frame, tf2::TimePointZero);
            
            hand_pose.position.x = transformStamped.transform.translation.x;
            hand_pose.position.y = transformStamped.transform.translation.y;
            hand_pose.position.z = transformStamped.transform.translation.z;
            hand_pose.orientation = transformStamped.transform.rotation;
           
            publisher_->publish(hand_pose);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", base_frame.c_str(), end_effector_frame.c_str(), ex.what());
        }
    }

    geometry_msgs::msg::Pose get_hand_pose() const { return hand_pose; }

private:
    std::string base_frame = "base_link";
    std::string end_effector_frame = "hand_cp"; 
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
    
    geometry_msgs::msg::Pose hand_pose;  

};

void handle_service(const std::shared_ptr<roarm_msgs::srv::GetPoseCmd::Request> request,
                    std::shared_ptr<roarm_msgs::srv::GetPoseCmd::Response> response,
                    std::shared_ptr<RobotPoseSubscription> node)
{
    double roll, pitch, yaw;
    node->update_end_effector_pose();

    geometry_msgs::msg::Pose pose = node->get_hand_pose();
    tf2::getEulerYPR<geometry_msgs::msg::Quaternion>(pose.orientation, yaw, pitch, roll);

    std::string model = get_roarm_model();
    response->x = format_to_6decimal(pose.position.x);
    response->y = format_to_6decimal(pose.position.y);
    if (model == "roarm_m2") {
      response->z = format_to_6decimal(pose.position.z-0.1338);
      response->roll = 0.0;
      response->pitch = 0.0;
      response->yaw = 0.0;
    } else if (model == "roarm_m3") {
      response->z = format_to_6decimal(pose.position.z-0.051959);
      response->roll = roll;
      response->pitch = pitch;
      response->yaw = yaw;
    }    
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotPoseSubscription>();
    
    auto server = node->create_service<roarm_msgs::srv::GetPoseCmd>("get_pose_cmd", 
                    std::bind(&handle_service, std::placeholders::_1, std::placeholders::_2, node));

    RCLCPP_INFO(node->get_logger(), "Service is ready to receive requests.");

    rclcpp::spin(node);  

    rclcpp::shutdown(); 

    return 0;
}
