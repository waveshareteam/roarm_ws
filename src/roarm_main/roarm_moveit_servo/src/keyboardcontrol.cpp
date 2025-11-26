/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*      Title     : servo_keyboard_input.cpp
 *      Project   : moveit_servo
 *      Created   : 05/31/2021
 *      Author    : Adam Pettinger, V Mohammed Ibrahim
 */

#include <chrono>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <roarm_msgs/srv/servo_command_type.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <stdio.h>
#include <iomanip>
#include <sstream>
#include <cstdlib>
#ifndef WIN32
#include <termios.h>
#include <unistd.h>
#else
#include <conio.h>
#endif

std::string get_roarm_model()
{
  const char *env_val = std::getenv("ROARM_MODEL");
  if (env_val == nullptr)
  {
    std::cerr << "no ROARM_MODEL!" << std::endl;
    return "";
  }
  return std::string(env_val);
}

std::string model = get_roarm_model();

// Define used keys
namespace
{
  constexpr int8_t KEYCODE_J = 0x6A;
  constexpr int8_t KEYCODE_T = 0x74;
  constexpr int8_t KEYCODE_W = 0x77;
  constexpr int8_t KEYCODE_E = 0x65;
  constexpr int8_t KEYCODE_S = 0x73;
  constexpr int8_t KEYCODE_1 = 0x31;
  constexpr int8_t KEYCODE_2 = 0x32;
  constexpr int8_t KEYCODE_3 = 0x33;
  constexpr int8_t KEYCODE_4 = 0x34;
  constexpr int8_t KEYCODE_5 = 0x35;
  constexpr int8_t KEYCODE_X = 0x78;
  constexpr int8_t KEYCODE_Y = 0x79;
  constexpr int8_t KEYCODE_Z = 0x7A;
  constexpr int8_t KEYCODE_R = 0x72;
  constexpr int8_t KEYCODE_P = 0x70;
  constexpr int8_t KEYCODE_G = 0x67;
  constexpr int8_t KEYCODE_Q = 0x71;
} // namespace

// Some constants used in the Servo Teleop demo
namespace
{
  const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
  const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
  const std::string GRIPPER_TOPIC = "/gripper_cmd";
  const size_t ROS_QUEUE_SIZE = 10;
  const std::string PLANNING_FRAME_ID = "base_link";
  const std::string EE_FRAME_ID = "hand_tcp";
} // namespace

// A class for reading the key inputs from the terminal
class KeyboardReader
{
public:
  KeyboardReader() : file_descriptor_(0)
  {
#ifndef WIN32
    // get the console in raw mode
    tcgetattr(file_descriptor_, &cooked_);
    struct termios raw;
    memcpy(&raw, &cooked_, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(file_descriptor_, TCSANOW, &raw);
#endif
  }
  void readOne(char *c)
  {
#ifndef WIN32
    int rc = read(file_descriptor_, c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }
#else
    *c = static_cast<char>(_getch());
#endif
  }
  void shutdown()
  {
#ifndef WIN32
    tcsetattr(file_descriptor_, TCSANOW, &cooked_);
#endif
  }

private:
  int file_descriptor_;
#ifndef WIN32
  struct termios cooked_;
#endif
};

// Converts key-presses to Twist or Jog commands for Servo, in lieu of a controller
class KeyboardServo
{
public:
  KeyboardServo();
  int keyLoop();

private:
  void spin();
  double gripper_value_;
  rclcpp::Node::SharedPtr nh_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Client<roarm_msgs::srv::ServoCommandType>::SharedPtr switch_input_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_pub_;

  std::shared_ptr<roarm_msgs::srv::ServoCommandType::Request> request_;
  double joint_vel_cmd_;
  double twist_vel_cmd_;
  std::string command_frame_id_;
};

KeyboardServo::KeyboardServo() : joint_vel_cmd_(1.0), twist_vel_cmd_(0.5), gripper_value_(0.0), command_frame_id_{"base_link"}
{
  nh_ = rclcpp::Node::make_shared("keyboard_control");

  twist_pub_ = nh_->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, ROS_QUEUE_SIZE);
  joint_pub_ = nh_->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, ROS_QUEUE_SIZE);
  gripper_pub_ = nh_->create_publisher<std_msgs::msg::Float32>(GRIPPER_TOPIC, ROS_QUEUE_SIZE);

  // Client for switching input types
  switch_input_ = nh_->create_client<roarm_msgs::srv::ServoCommandType>("servo_node/switch_command_type");
}

KeyboardReader input;

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  KeyboardServo keyboard_servo;

  signal(SIGINT, quit);

  int rc = keyboard_servo.keyLoop();
  input.shutdown();
  rclcpp::shutdown();

  return rc;
}

void KeyboardServo::spin()
{
  while (rclcpp::ok())
  {
    rclcpp::spin_some(nh_);
  }
}

int KeyboardServo::keyLoop()
{
  char c;
  bool publish_twist = false;
  bool publish_joint = false;
  bool publish_gripper = false;

  std::thread{[this]()
              { return spin(); }}
      .detach();

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("All commands are in the planning frame");
  puts("Use 'j' to select joint jog. ");
  if (model == "roarm_m2")
  {
    puts("Use 1|2|3| keys to joint jog. 'g' to control gripper; 's' to reverse the direction of jogging.");
  }
  else if (model == "roarm_m3")
  {
    puts("Use 1|2|3|4|5| keys to joint jog. 'g' to control gripper; 's' to reverse the direction of jogging.");
  }
  puts("Use 't' to select twist ");
  puts("Use 'w' and 'e' to switch between sending command in planning frame or end effector frame");
  if (model == "roarm_m2")
  {
    puts("Use x|y|z| keys to Cartesian jog. 's' to reverse the direction of twist.");
  }
  else if (model == "roarm_m3")
  {
    puts("Use x|y|z|r|p| keys to Cartesian jog. 's' to reverse the direction of twist.");
  }
  puts("'Q' to quit.");

  for (;;)
  {
    // get the next event from the keyboard
    try
    {
      input.readOne(&c);
    }
    catch (const std::runtime_error &)
    {
      perror("read():");
      return -1;
    }

    // // Create the messages we might publish
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();
    auto gripper_msg = std::make_unique<std_msgs::msg::Float32>();

    if (model == "roarm_m2")
    {
      joint_msg->joint_names.resize(3);
      joint_msg->joint_names = {"base_link_to_link1", "link1_to_link2", "link2_to_link3"};
    }
    else if (model == "roarm_m3")
    {
      joint_msg->joint_names.resize(5);
      joint_msg->joint_names = {"base_link_to_link1", "link1_to_link2", "link2_to_link3", "link3_to_link4", "link4_to_link5"};
    }
    joint_msg->velocities.resize(7);
    std::fill(joint_msg->velocities.begin(), joint_msg->velocities.end(), 0.0);
    // Use read key-press
    switch (c)
    {
    case KEYCODE_S:
      RCLCPP_DEBUG(nh_->get_logger(), "s");
      joint_vel_cmd_ *= -1;
      twist_vel_cmd_ *= -1;
      break;
    case KEYCODE_1:
      RCLCPP_DEBUG(nh_->get_logger(), "1");
      joint_msg->velocities[0] = joint_vel_cmd_;
      publish_joint = true;
      break;
    case KEYCODE_2:
      RCLCPP_DEBUG(nh_->get_logger(), "2");
      joint_msg->velocities[1] = joint_vel_cmd_;
      publish_joint = true;
      break;
    case KEYCODE_3:
      RCLCPP_DEBUG(nh_->get_logger(), "3");
      joint_msg->velocities[2] = joint_vel_cmd_;
      publish_joint = true;
      break;
    case KEYCODE_4:
      RCLCPP_DEBUG(nh_->get_logger(), "4");
      joint_msg->velocities[3] = joint_vel_cmd_;
      publish_joint = true;
      break;
    case KEYCODE_5:
      RCLCPP_DEBUG(nh_->get_logger(), "5");
      joint_msg->velocities[4] = joint_vel_cmd_;
      publish_joint = true;
      break;      
    case KEYCODE_X:
      RCLCPP_DEBUG(nh_->get_logger(), "x");
      twist_msg->twist.linear.x = twist_vel_cmd_;
      publish_twist = true;
      break;      
    case KEYCODE_Y:
      RCLCPP_DEBUG(nh_->get_logger(), "y");
      twist_msg->twist.linear.y = twist_vel_cmd_;
      publish_twist = true;
      break;
    case KEYCODE_Z:
      RCLCPP_DEBUG(nh_->get_logger(), "z");
      twist_msg->twist.linear.z = twist_vel_cmd_;
      publish_twist = true;
      break;      
    case KEYCODE_R:
      if (model == "roarm_m3")
      {
        RCLCPP_DEBUG(nh_->get_logger(), "r");
        twist_msg->twist.angular.x = twist_vel_cmd_;
        publish_twist = true;
      }
      break;      
    case KEYCODE_P:
      if (model == "roarm_m3")
      {
        RCLCPP_DEBUG(nh_->get_logger(), "p");
        twist_msg->twist.angular.y = twist_vel_cmd_;
        publish_twist = true;
      }
      break;      
    case KEYCODE_G:
      RCLCPP_DEBUG(nh_->get_logger(), "g");
      if (joint_vel_cmd_ > 0)
      {
        gripper_value_ += 0.01;
      }
      else if (joint_vel_cmd_ < 0)
      {
        gripper_value_ -= 0.01;
      }
      if (gripper_value_ > 1.5)
      {
        gripper_value_ = 1.5;
        puts("MAX 1.5");
      }
      else if (gripper_value_ < 0.0)
      {
        gripper_value_ = 0.0;
        puts("MIN 0,0");
      }
      gripper_msg->data = gripper_value_;
      publish_gripper = true;
      break;
    case KEYCODE_J:
      RCLCPP_DEBUG(nh_->get_logger(), "j");
      request_ = std::make_shared<roarm_msgs::srv::ServoCommandType::Request>();
      request_->command_type = roarm_msgs::srv::ServoCommandType::Request::JOINT_JOG;
      if (switch_input_->wait_for_service(std::chrono::seconds(1)))
      {
        auto result = switch_input_->async_send_request(request_);
        if (result.get()->success)
        {
          RCLCPP_INFO_STREAM(nh_->get_logger(), "Switched to input type: JointJog");
        }
        else
        {
          RCLCPP_WARN_STREAM(nh_->get_logger(), "Could not switch input to: JointJog");
        }
      }
      break;
    case KEYCODE_T:
      RCLCPP_DEBUG(nh_->get_logger(), "t");
      request_ = std::make_shared<roarm_msgs::srv::ServoCommandType::Request>();
      request_->command_type = roarm_msgs::srv::ServoCommandType::Request::TWIST;
      if (switch_input_->wait_for_service(std::chrono::seconds(1)))
      {
        auto result = switch_input_->async_send_request(request_);
        if (result.get()->success)
        {
          RCLCPP_INFO_STREAM(nh_->get_logger(), "Switched to input type: Twist");
        }
        else
        {
          RCLCPP_WARN_STREAM(nh_->get_logger(), "Could not switch input to: Twist");
        }
      }
      break;
    case KEYCODE_W:
      RCLCPP_DEBUG(nh_->get_logger(), "w");
      RCLCPP_INFO_STREAM(nh_->get_logger(), "Command frame set to: " << PLANNING_FRAME_ID);
      command_frame_id_ = PLANNING_FRAME_ID;
      break;
    case KEYCODE_E:
      RCLCPP_DEBUG(nh_->get_logger(), "e");
      RCLCPP_INFO_STREAM(nh_->get_logger(), "Command frame set to: " << EE_FRAME_ID);
      command_frame_id_ = EE_FRAME_ID;
      break;
    case KEYCODE_Q:
      RCLCPP_DEBUG(nh_->get_logger(), "quit");
      return 0;
    }

    // If a key requiring a publish was pressed, publish the message now

    if (publish_twist)
    {
      twist_msg->header.stamp = nh_->now();
      twist_msg->header.frame_id = command_frame_id_;
      twist_pub_->publish(std::move(twist_msg));
      publish_twist = false;
    }
    if (publish_joint)
    {
      joint_msg->header.stamp = nh_->now();
      joint_msg->header.frame_id = PLANNING_FRAME_ID;
      joint_pub_->publish(std::move(joint_msg));
      publish_joint = false;
    }
    if (publish_gripper)
    {
      gripper_pub_->publish(std::move(gripper_msg));
      publish_gripper = false;
    }
  }

  return 0;
}
