/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019 PickNik LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Henning Kayser, Simon Goldstein
  Desc:   A demo to show MoveIt Task Constructor in action
*/

// ROS
#include <rclcpp/node.hpp>

// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// MTC
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
#include "pick_place_parameters.hpp"
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
#include <iostream>
#include <cmath>
#include <iomanip>
#include <sstream> 
#include <cstdlib> 

#pragma once

class YoloPoseSubscription : public rclcpp::Node
{
public:
  YoloPoseSubscription() : Node("yolo_pose_subscription")
  {
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  geometry_msgs::msg::Pose update_yolo_pose() 
  {
      try
      {
          geometry_msgs::msg::TransformStamped transformStamped;
          transformStamped = tf_buffer_->lookupTransform(base_frame, yolo_frame, tf2::TimePointZero);

          
          yolo_pose.position.x = transformStamped.transform.translation.x;
          yolo_pose.position.y = transformStamped.transform.translation.y;
          yolo_pose.position.z = transformStamped.transform.translation.z;

          yolo_pose.orientation = transformStamped.transform.rotation;
      }
      catch (tf2::TransformException &ex)
      {
          RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", base_frame.c_str(), yolo_frame.c_str(), ex.what());
      }
      return yolo_pose;
  }

private:
  std::string base_frame = "world";
  std::string yolo_frame = "object_1"; 
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  
  geometry_msgs::msg::Pose yolo_pose;  

};

namespace roarm_moveit_mtc_demo {
using namespace moveit::task_constructor;

// prepare a demo environment from ROS parameters under node
void setupDemoScene(const std::shared_ptr<YoloPoseSubscription>& yolo_node, const pick_place_task_demo::Params& params);

class PickPlaceTask
{
public:
	PickPlaceTask(const std::string& task_name);
	~PickPlaceTask() = default;

	bool init(const rclcpp::Node::SharedPtr& node, const pick_place_task_demo::Params& params);

	bool plan(const std::size_t max_solutions);

	bool execute();

private:
	std::string task_name_;
	moveit::task_constructor::TaskPtr task_;
};
}  // namespace roarm_moveit_mtc_demo
