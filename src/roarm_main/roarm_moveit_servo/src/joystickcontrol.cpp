/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
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
 *   * Neither the name of PickNik Inc. nor the names of its
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

/*      Title     : joystick_servo_example.cpp
 *      Project   : moveit_servo
 *      Created   : 08/07/2020
 *      Author    : Adam Pettinger
 */

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <roarm_msgs/srv/servo_command_type.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/qos_event.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <thread>

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

#include <SDL2/SDL.h>
#include <iostream>
#include <vector>
#include <string>

std::vector<std::string> getJoystickNames()
{
  std::vector<std::string> names;

  if (SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_GAMECONTROLLER) < 0)
  {
    std::cerr << "Failed to initialize SDL: " << SDL_GetError() << std::endl;
    return names;
  }

  int num_joysticks = SDL_NumJoysticks();
  if (num_joysticks == 0)
  {
    std::cout << "No joystick detected." << std::endl;
  }
  else
  {
    for (int i = 0; i < num_joysticks; ++i)
    {
      const char *name = SDL_JoystickNameForIndex(i);
      if (name)
        names.emplace_back(name);
    }
  }

  SDL_Quit();
  return names;
}

std::vector<std::string> joystick_names = getJoystickNames();

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

// We'll just set up parameters here
const std::string JOY_TOPIC = "/joy";
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const std::string GRIPPER_TOPIC = "/gripper_cmd";
const std::string EEF_FRAME_ID = "hand_tcp";
const std::string BASE_FRAME_ID = "base_link";

struct JoystickMapping
{
  int LEFT_STICK_X;
  int LEFT_STICK_Y;
  int RIGHT_STICK_X;
  int RIGHT_STICK_Y;
  int LEFT_TRIGGER;
  int RIGHT_TRIGGER;
  int D_PAD_X;
  int D_PAD_Y;

  int A;
  int B;
  int X;
  int Y;
  int LEFT_BUMPER;
  int RIGHT_BUMPER;
  int SELECT;
  int START;
  int HOME;
  int LEFT_STICK_CLICK;
  int RIGHT_STICK_CLICK;
};

JoystickMapping SHANWAN_Android_Gamepad = {
    0, 1, 2, 3, 5, 4, 6, 7,              // axes
    0, 1, 3, 4, 6, 7, 10, 11, 12, 13, 14 // buttons
};

JoystickMapping Xbox_360_Controller = {
    0, 1, 3, 4, 2, 5, 6, 7,
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

JoystickMapping getJoystickMapping(const std::string &name)
{
  if (name == "SHANWAN Android Gamepad")
  {
    return SHANWAN_Android_Gamepad;
  }
  else if (name == "Xbox 360 Controller")
  {
    return Xbox_360_Controller;
  }
  else
  {
    std::cerr << "Unknown joystick type: " << name << std::endl;
    return Xbox_360_Controller; 
  }
}

std::string name = joystick_names.empty() ? "Xbox 360 Controller" : joystick_names[0];
JoystickMapping mapping = getJoystickMapping(name);

double initial_left_stick_x = 0.0;
double initial_left_stick_y = 0.0;
double initial_right_stick_x = 0.0;
double initial_right_stick_y = 0.0;
bool joystick_calibrated = false;

void recordInitialJoystickValues(const std::vector<float> &axes)
{
  initial_left_stick_x = axes[mapping.LEFT_STICK_X];
  initial_left_stick_y = axes[mapping.LEFT_STICK_Y];
  initial_right_stick_x = axes[mapping.RIGHT_STICK_X];
  initial_right_stick_y = axes[mapping.RIGHT_STICK_Y];
  joystick_calibrated = true;
}

double gripper_value_ = 0.0;
const double gripper_threshold = 0.01;
// To change controls or setup a new controller, all you should to do is change the above enums and the follow 2
// functions
/** \brief // This converts a joystick axes and buttons array to a TwistStamped or JointJog message
 * @param axes The vector of continuous controller joystick axes
 * @param buttons The vector of discrete controller button values
 * @param twist_msg A TwistStamped message to update in prep for publishing
 * @param joint_msg A JointJog message to update in prep for publishing
 * @param gripper_msg A Float32 message to update in prep for publishing
 * @return return true if you want to publish a Twist, false if you want to publish a JointJog
 */
int convertJoyToCmd(const std::vector<float> &axes,
                    const std::vector<int> &buttons,
                    std::unique_ptr<geometry_msgs::msg::TwistStamped> &twist_msg,
                    std::unique_ptr<control_msgs::msg::JointJog> &joint_msg,
                    std::unique_ptr<std_msgs::msg::Float32> &gripper_msg)
{
  double left_stick_x, left_stick_y, right_stick_x, right_stick_y;
  if (!joystick_calibrated)
  {
    recordInitialJoystickValues(axes);
  }

  left_stick_x = axes[mapping.LEFT_STICK_X] - initial_left_stick_x;
  left_stick_y = axes[mapping.LEFT_STICK_Y] - initial_left_stick_y;
  right_stick_x = axes[mapping.RIGHT_STICK_X] - initial_right_stick_x;
  right_stick_y = axes[mapping.RIGHT_STICK_Y] - initial_right_stick_y;

  if (joystick_calibrated)
  {
    if (buttons[mapping.RIGHT_BUMPER])
    {
      twist_msg->twist.linear.x = left_stick_y;
      twist_msg->twist.linear.y = left_stick_x;

      twist_msg->twist.angular.x = -right_stick_x;
      twist_msg->twist.angular.y = right_stick_y;

      if (axes[mapping.RIGHT_TRIGGER] == 1)
      {
        twist_msg->twist.linear.z = 1 * buttons[mapping.LEFT_STICK_CLICK];
      }
      else
      {
        twist_msg->twist.linear.z = -1 * buttons[mapping.LEFT_STICK_CLICK];
      }

      return 0;
    }

    if (!buttons[mapping.RIGHT_BUMPER])
    {
      if (axes[mapping.LEFT_STICK_X] || axes[mapping.LEFT_STICK_Y] || axes[mapping.RIGHT_STICK_X] || axes[mapping.RIGHT_STICK_Y] || buttons[mapping.LEFT_STICK_CLICK] )
      {
        if (model == "roarm_m2")
        {
          joint_msg->joint_names.resize(3);
          joint_msg->joint_names = {"base_link_to_link1", "link1_to_link2", "link2_to_link3"};
          joint_msg->velocities.resize(3);
          joint_msg->velocities[0] = left_stick_x;
          joint_msg->velocities[1] = left_stick_y;
          if (axes[mapping.RIGHT_TRIGGER] == 1)
          {
            joint_msg->velocities[2] = -1 * buttons[mapping.LEFT_STICK_CLICK];
          }
          else
          {
            joint_msg->velocities[2] = buttons[mapping.LEFT_STICK_CLICK];
          }
        }
        else if (model == "roarm_m3")
        {
          joint_msg->joint_names.resize(5);
          joint_msg->joint_names = {"base_link_to_link1", "link1_to_link2", "link2_to_link3", "link3_to_link4", "link4_to_link5"};
          joint_msg->velocities.resize(5);
          joint_msg->velocities[0] = left_stick_x;
          joint_msg->velocities[1] = left_stick_y;
          if (axes[mapping.RIGHT_TRIGGER] == 1)
          {
            joint_msg->velocities[2] = -1 * buttons[mapping.LEFT_STICK_CLICK];
          }
          else
          {
            joint_msg->velocities[2] = buttons[mapping.LEFT_STICK_CLICK];
          }
          joint_msg->velocities[3] = right_stick_y;
          joint_msg->velocities[4] = -right_stick_x;
        }

        return 1;
      }
    }

    if (buttons[mapping.A] || buttons[mapping.B])
    {
      if (buttons[mapping.B])
      {
        gripper_value_ += 0.01;
        if (gripper_value_ > 1.5)
        {
          gripper_value_ = 1.5;
          puts("MAX 1.5");
        }
      }
      if (buttons[mapping.A])
      {
        gripper_value_ -= 0.01;
        if (gripper_value_ < 0.0)
        {
          gripper_value_ = 0.0;
          puts("MIN 0,0");
        }  
      }
      gripper_msg->data = gripper_value_;
      return 2;
    }
  }
}

/** \brief // This should update the frame_to_publish_ as needed for changing command frame via controller
 * @param frame_name Set the command frame to this
 * @param buttons The vector of discrete controller button values
 */
void updateCmdFrame(std::string &frame_name, const std::vector<int> &buttons)
{
  if (buttons[mapping.X] && frame_name == EEF_FRAME_ID)
    frame_name = BASE_FRAME_ID;
  else if (buttons[mapping.Y] && frame_name == BASE_FRAME_ID)
    frame_name = EEF_FRAME_ID;
}

namespace roarm_moveit_servo
{
  class JoyToServoPub : public rclcpp::Node
  {
  public:
    JoyToServoPub(const rclcpp::NodeOptions &options)
        : Node("joy_to_twist_publisher", options), frame_to_publish_(BASE_FRAME_ID)
    {
      // Setup pub/sub
      joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
          JOY_TOPIC, rclcpp::SystemDefaultsQoS(),
          [this](const sensor_msgs::msg::Joy::ConstSharedPtr &msg)
          { return joyCB(msg); });

      twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, rclcpp::SystemDefaultsQoS());
      joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, rclcpp::SystemDefaultsQoS());
      gripper_pub_ = this->create_publisher<std_msgs::msg::Float32>(GRIPPER_TOPIC, rclcpp::SystemDefaultsQoS());

      // Create a service client to start the ServoNode
      servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
      servo_start_client_->wait_for_service(std::chrono::seconds(1));
      servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      switch_input_ = this->create_client<roarm_msgs::srv::ServoCommandType>("servo_node/switch_command_type");
    }

    ~JoyToServoPub() override
    {
    }

    void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr &msg)
    {
      if(!msg){return;}
      // Create the messages we might publish
      auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
      auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();
      auto gripper_msg = std::make_unique<std_msgs::msg::Float32>();

      int current_right_bumper = msg->buttons[mapping.RIGHT_BUMPER];

      int8_t new_command_type = (current_right_bumper != 0) ?
          roarm_msgs::srv::ServoCommandType::Request::TWIST :
          roarm_msgs::srv::ServoCommandType::Request::JOINT_JOG;

      if (current_right_bumper != last_right_bumper_)
      {
        auto request = std::make_shared<roarm_msgs::srv::ServoCommandType::Request>();
        request->command_type = new_command_type;

        if (switch_input_->wait_for_service(std::chrono::milliseconds(10)))
        {
          switch_input_->async_send_request(request);
        }

        last_command_type_ = new_command_type;
      }

      last_right_bumper_ = current_right_bumper;

      // This call updates the frame for twist commands
      updateCmdFrame(frame_to_publish_, msg->buttons);

      int choose = convertJoyToCmd(msg->axes, msg->buttons, twist_msg, joint_msg, gripper_msg);
      // Convert the joystick message to Twist or JointJog and publish
      if (choose == 0)
      {
        // publish the TwistStamped
        twist_msg->header.frame_id = frame_to_publish_;
        twist_msg->header.stamp = this->now();
        twist_pub_->publish(std::move(twist_msg));
      }
      if (choose == 1)
      {
        // publish the JointJog
        joint_msg->header.stamp = this->now();
        joint_msg->header.frame_id = "base_link";
        joint_pub_->publish(std::move(joint_msg));
      }

      if (choose == 2)
      {
        double current_value = gripper_msg->data;

        if (std::abs(current_value - last_gripper_value) > gripper_threshold)
        {
          gripper_pub_->publish(std::move(gripper_msg));
          last_gripper_value = current_value;
        }
      }
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
    rclcpp::Client<roarm_msgs::srv::ServoCommandType>::SharedPtr switch_input_;
    std::shared_ptr<roarm_msgs::srv::ServoCommandType::Request> request_;

    std::string frame_to_publish_;
    int8_t last_command_type_ = -1;
    double last_gripper_value = 0.0;
    int last_right_bumper_ = 0;

  }; // class JoyToServoPub

} // namespace roarm_moveit_servo

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(roarm_moveit_servo::JoyToServoPub)
