// Copyright 2021 RobosoftAI Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <angles/angles.h>
#include <geometry_msgs/msg/twist.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>
#include <sm_nav2_test_9/clients/cl_nav2z/client_behaviors/cb_position_control_free_space.hpp>

namespace sm_nav2_test_9
{
CbPositionControlFreeSpace::CbPositionControlFreeSpace()
: targetYaw_(0), k_betta_(1.0), max_angular_yaw_speed_(1.0)
{
}

void CbPositionControlFreeSpace::updateParameters() {}

void CbPositionControlFreeSpace::onEntry()
{
  auto nh = this->getNode();
  cmd_vel_pub_ = nh->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(1));

      this->max_linear_velocity = 0.1;
      if(!getNode()->has_parameter("cb_battery_position_control.max_linear_velocity"))
      {
        getNode()->declare_parameter("cb_battery_position_control.max_linear_velocity",this->max_linear_velocity);
      }
      this->max_linear_velocity = getNode()->get_parameter("cb_battery_position_control.max_linear_velocity").as_double();

      if(!getNode()->has_parameter("cb_battery_position_control.max_angular_velocity"))
      {
        getNode()->declare_parameter("cb_battery_position_control.max_angular_velocity",this->max_angular_velocity);
      }
      this->max_angular_velocity = getNode()->get_parameter("cb_battery_position_control.max_angular_velocity").as_double();

      this->threshold_distance_ = 0.05;
      if(!getNode()->has_parameter("cb_battery_position_control.threshold_distance"))
      {
        getNode()->declare_parameter("cb_battery_position_control.threshold_distance",this->threshold_distance_);
      }
      this->threshold_distance_ = getNode()->get_parameter("cb_battery_position_control.threshold_distance").as_double();

      this->kp_linear = 0.5;
      if (!getNode()->has_parameter("cb_battery_position_control.kp_linear"))
      {
        getNode()->declare_parameter("cb_battery_position_control.kp_linear", this->kp_linear);
      }
      this->kp_linear = getNode()->get_parameter("cb_battery_position_control.kp_linear").as_double();

      this->kp_lateral = 0.5;
      if (!getNode()->has_parameter("cb_battery_position_control.kp_lateral"))
      {
        getNode()->declare_parameter("cb_battery_position_control.kp_lateral", this->kp_lateral);
      }
      this->kp_lateral = getNode()->get_parameter("cb_battery_position_control.kp_lateral").as_double();
      
      this->kp_angular = 0.0; // 0.5
      if (!getNode()->has_parameter("cb_battery_position_control.kp_angular"))
      {
        getNode()->declare_parameter("cb_battery_position_control.kp_angular", this->kp_angular);
      }
      this->kp_angular = getNode()->get_parameter("cb_battery_position_control.kp_angular").as_double();

      this->kd_angular=0.0;
      if (!getNode()->has_parameter("cb_battery_position_control.kd_angular"))
      {
        getNode()->declare_parameter("cb_battery_position_control.kd_angular", this->kd_angular);
      }
      this->kd_angular = getNode()->get_parameter("cb_battery_position_control.kd_angular").as_double();

      this->kp_angular_betta=1.0;
      if (!getNode()->has_parameter("cb_battery_position_control.kp_angular_betta"))
      {
        getNode()->declare_parameter("cb_battery_position_control.kp_angular_betta", this->kp_angular_betta);
      }
      this->kp_angular_betta = getNode()->get_parameter("cb_battery_position_control.kp_angular_betta").as_double();

      this->command_timeout_sec = 10.0;
      if (!getNode()->has_parameter("cb_battery_position_control.command_timeout_sec"))
      {
        getNode()->declare_parameter("cb_battery_position_control.command_timeout_sec", this->command_timeout_sec);
      }
      this->command_timeout_sec = getNode()->get_parameter("cb_battery_position_control.command_timeout_sec").as_double();


      
      // RCLCPP_INFO(getLogger(), "Setting target pose for docking: %f, %f", target_pose_.position.x, target_pose_.position.y);

      // RCLCPP_INFO(getLogger(), "Setting kd angular: %lf", this->kd_angular);



  cl_nav2z::Pose * pose;
  this->requiresComponent(pose);

  geometry_msgs::msg::Twist cmd_vel;
  goalReached_ = false;

  geometry_msgs::msg::Pose currentPose = pose->toPoseMsg();

  rclcpp::Rate loop_rate(10);
  double countAngle = 0;
  auto prevyaw = tf2::getYaw(currentPose.orientation);

  commandStartTime_ = this->getNode()->get_clock()->now();       

  geometry_msgs::msg::Pose target_pose_;                                   

  bool timeout = false;

  while (rclcpp::ok() && !goalReached_)
  {
    
    auto target_pose_opt = this->getTargetPoseCallback_();

    if (target_pose_opt)
    target_pose_ = *target_pose_opt;

    tf2::Quaternion q;
    currentPose = pose->toPoseMsg();


    // Calculate the errors in x and y
    double error_x = target_pose_.position.x - currentPose.position.x;
    double error_y = target_pose_.position.y - currentPose.position.y;

    // Calculate the distance to the target pose
    // double distance_to_target = std::sqrt(error_x * error_x + error_y * error_y);
    double distance_to_target = std::sqrt(error_x * error_x + error_y * error_y);

    double alpha = atan2(error_y, error_x) - tf2::getYaw(currentPose.orientation);


    //final orientation error
    double betta = - tf2::getYaw(currentPose.orientation) - alpha;

    RCLCPP_INFO_STREAM(
      getLogger(), "[" << getName() << "] ----- CONTROL LOOP ----- ");

      RCLCPP_INFO(getLogger(), "Setting max linear velocity: %lf", this->max_linear_velocity);
      RCLCPP_INFO(getLogger(), "Setting max angular velocity: %lf", this->max_angular_velocity);
      RCLCPP_INFO(getLogger(), "Setting threshold distance: %lf", this->threshold_distance_);
      RCLCPP_INFO(getLogger(), "Setting kp linear: %lf", this->kp_linear);
      RCLCPP_INFO(getLogger(), "Setting kp lateral: %lf", this->kp_lateral);

      RCLCPP_INFO(getLogger(), "Setting kp angular: %lf", this->kp_angular);
      RCLCPP_INFO(getLogger(), "Setting kd angular: %lf", this->kd_angular);
      RCLCPP_INFO(getLogger(), "Setting ki angular: %lf", this->ki_angular);

      RCLCPP_INFO(getLogger(), "Setting kp angular betta: %lf", this->kp_angular_betta);
      RCLCPP_INFO(getLogger(), "Setting kd angular betta: %lf", this->kd_angular_betta);
      RCLCPP_INFO(getLogger(), "Setting ki angular betta: %lf", this->ki_angular_betta);
      

    RCLCPP_INFO_STREAM(
      getLogger(), "[" << getName() << "] --- ");

    RCLCPP_INFO_STREAM(
      getLogger(), "[" << getName() << "] distance to target: " << distance_to_target
                       << " ( th: " << threshold_distance_ << ")");

    RCLCPP_INFO_STREAM(
      getLogger(), "[" << getName() << "] linearerror x: " << error_x << ", y: " << error_y);

    RCLCPP_INFO_STREAM(
      getLogger(), "[" << getName() << "] final orientation error (betta): " << betta);

      RCLCPP_INFO_STREAM(
        getLogger(), "[" << getName() << "] robot orientation: " << tf2::getYaw(currentPose.orientation));
      
      auto absolute_alpha = std::atan2(error_y, error_x) ;

      RCLCPP_INFO_STREAM(
        getLogger(), "[" << getName() << "] bearing error (alpha): " << alpha);
      

    // print robot pose and target pose
    RCLCPP_INFO_STREAM(
      getLogger(), "[" << getName() << "] current pose: " << currentPose.position.x << ", "
                       << currentPose.position.y << ", " << tf2::getYaw(currentPose.orientation));
    RCLCPP_INFO_STREAM(
      getLogger(), "[" << getName() << "] target pose: " << target_pose_.position.x << ", "
                       << target_pose_.position.y << ", " << tf2::getYaw(target_pose_.orientation));

    // Check if the robot has reached the target pose
    if (distance_to_target < threshold_distance_)
    {
      RCLCPP_INFO(getLogger(), "Goal reached!");
      // Stop the robot by setting the velocity commands to zero
      geometry_msgs::msg::Twist cmd_vel_msg;
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd_vel_msg);
      break;
    }
    else
    {
      // Calculate the desired orientation angle
      double alpha_error = alpha;

      // Calculate the difference between the desired orientation and the current orientation
      // double yaw_error = desired_yaw - (tf2::getYaw(currentPose.orientation) + M_PI );

      // Ensure the yaw error is within the range [-pi, pi]
      // while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
      // while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

      // Calculate the control signals (velocity commands) using PID controllers
      double cmd_linear_x = kp_linear * distance_to_target + ki_linear * integral_linear_ +
                            kd_linear * (distance_to_target - prev_error_linear_);

      double cmd_angular_z = kp_angular * alpha_error 
                             + ki_angular * integral_angular_ 
                             + kd_angular * (alpha_error - prev_error_angular_)

                             + kp_angular_betta * betta
                             + ki_angular_betta * integral_betta_
                             + kd_angular_betta * (betta - prev_error_betta_)

                             + kp_lateral * error_y
                             ;


       // print betta and alpha angular.z contributions
      RCLCPP_INFO_STREAM(
        getLogger(), "[" << getName() << "] betta angular.z contribution: " << kp_angular_betta * betta);

      RCLCPP_INFO_STREAM(
        getLogger(), "[" << getName() << "] alpha angular.z contribution: " << kp_angular * alpha_error);

      RCLCPP_INFO_STREAM(
        getLogger(), "[" << getName() << "] error y angular.z contribution: " << kp_linear * error_y);
      


      if (cmd_linear_x > max_linear_velocity)
        cmd_linear_x = max_linear_velocity;
      else if (cmd_linear_x < -max_linear_velocity)
        cmd_linear_x = -max_linear_velocity;

      if (cmd_angular_z > max_angular_velocity)
        cmd_angular_z = max_angular_velocity;
      else if (cmd_angular_z < -max_angular_velocity)
        cmd_angular_z = -max_angular_velocity;

      // Construct and publish the velocity command message
      geometry_msgs::msg::Twist cmd_vel_msg;

      cmd_vel_msg.linear.x = cmd_linear_x;
      cmd_vel_msg.angular.z = cmd_angular_z;

      RCLCPP_INFO_STREAM(
        getLogger(), "[" << getName() << "] cmd_vel linear: " << cmd_vel_msg.linear.x
                         << ", angular: " << cmd_vel_msg.angular.z);

      
      cmd_vel_pub_->publish(cmd_vel_msg);

      // Update errors and integrals for the next control cycle
      prev_error_linear_ = distance_to_target;
      prev_error_angular_ = alpha_error;

      integral_linear_ += distance_to_target;
      integral_angular_ += alpha_error;

      integral_betta_ += betta;
      prev_error_betta_ = betta;

      // tf2::fromMsg(currentPose.orientation, q);
      // auto currentYaw = tf2::getYaw(currentPose.orientation);
      // auto deltaAngle = angles::shortest_angular_distance(prevyaw, currentYaw);
      // countAngle += deltaAngle;

      // prevyaw = currentYaw;
      // double angular_error = targetYaw_ - countAngle;

      // auto omega = angular_error * k_betta_;
      // cmd_vel.linear.x = 0;
      // cmd_vel.linear.y = 0;
      // cmd_vel.linear.z = 0;
      // cmd_vel.angular.z =
      //   std::min(std::max(omega, -fabs(max_angular_yaw_speed_)), fabs(max_angular_yaw_speed_));

      // RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] delta angle: " << deltaAngle);
      // RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] cummulated angle: " << countAngle);
      // RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] k_betta_: " << k_betta_);

      // RCLCPP_INFO_STREAM(
      //   getLogger(), "[" << getName() << "] angular error: " << angular_error << "("
      //                    << yaw_goal_tolerance_rads_ << ")");
      // RCLCPP_INFO_STREAM(
      //   getLogger(), "[" << getName() << "] command angular speed: " << cmd_vel.angular.z);

      // if (fabs(angular_error) < yaw_goal_tolerance_rads_)
      // {
      //   RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] GOAL REACHED. Sending stop command.");
      //   goalReached_ = true;
      //   cmd_vel.linear.x = 0;
      //   cmd_vel.angular.z = 0;
      //   break;
      // }

      // this->cmd_vel_pub_->publish(cmd_vel);

      auto ellapsedTime = this->getNode()->now() - commandStartTime_;
      if(ellapsedTime.seconds() > this->command_timeout_sec)
      {
        RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] COMMAND FINISHED (TIMEOUT). Sending stop command.");
        goalReached_ = true;
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        break;
      }

      loop_rate.sleep();
    }
  }

  RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] Finished behavior execution");

  this->postSuccessEvent();
}

void CbPositionControlFreeSpace::onExit() {}

}  // namespace sm_dancebot_mine_ue