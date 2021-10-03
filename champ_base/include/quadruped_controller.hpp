/*
Copyright (c) 2019-2020, Juan Miguel Jimeno
Copyright (c) 2021, Fetullah Atas
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef QUADRUPED_CONTROLLER_H
#define QUADRUPED_CONTROLLER_H

#include "rclcpp/rclcpp.hpp"

#include <champ_msgs/msg/joints.hpp>
#include <champ_msgs/msg/pose.hpp>
#include <champ_msgs/msg/point_array.hpp>
#include <champ_msgs/msg/contacts_stamped.hpp>

//#include <champ/utils/urdf_loader.h>
#include <champ/quadruped_base/quadruped_components.h>
#include <champ/quadruped_base/quadruped_base.h>
#include <champ/body_controller/body_controller.h>
#include <champ/leg_controller/leg_controller.h>
#include <champ/kinematics/kinematics.h>
#include <champ/geometry/geometry.h>
#include <champ/leg_controller/phase_generator.h>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <urdf/model.h>


class QuadrupedController : public rclcpp::Node
{
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr cmd_pose_subscriber_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_commands_publisher_;
  rclcpp::Publisher<champ_msgs::msg::ContactsStamped>::SharedPtr foot_contacts_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Node::SharedPtr parameter_client_node_;

  champ::Velocities req_vel_;
  champ::Pose req_pose_;

  champ::GaitConfig gait_config_;

  champ::QuadrupedBase base_;
  champ::BodyController body_controller_;
  champ::LegController leg_controller_;
  champ::Kinematics kinematics_;

  std::vector<std::string> joint_names_;

  bool publish_foot_contacts_;
  bool publish_joint_states_;
  bool publish_joint_control_;
  bool in_gazebo_;

  void controlLoop();

  void publishJoints(float target_joints[12]);
  void publishFootContacts(bool foot_contacts[4]);

  void cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
  void cmdPoseCallback(const geometry_msgs::msg::Pose::ConstSharedPtr msg);

  void getPose(urdf::Pose * pose, std::string ref_link, std::string end_link, urdf::Model & model)
  {
    urdf::LinkConstSharedPtr ref_link_ptr = model.getLink(ref_link);

    std::string current_parent_name = end_link;
    urdf::LinkConstSharedPtr prev_link = model.getLink(current_parent_name);

    while (ref_link_ptr->name != current_parent_name) {
      urdf::LinkConstSharedPtr current_link = model.getLink(current_parent_name);
      urdf::Pose current_pose = current_link->parent_joint->parent_to_joint_origin_transform;

      current_parent_name = current_link->getParent()->name;
      prev_link = model.getLink(current_parent_name);
      pose->position.x += current_pose.position.x;
      pose->position.y += current_pose.position.y;
      pose->position.z += current_pose.position.z;
    }
  }

public:
  QuadrupedController();
};

#endif
