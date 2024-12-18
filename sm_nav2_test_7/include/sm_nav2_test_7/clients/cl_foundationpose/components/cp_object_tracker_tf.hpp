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

#pragma once
#include <smacc2/component.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <smacc2/client_base_components/cp_topic_subscriber.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace cl_foundationpose 
{

using namespace smacc2::components;


class CpObjectTrackerTf : public smacc2::ISmaccComponent 
{

private:
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
  std::string global_frame_id_;

public:
  CpObjectTrackerTf() {}
  void onInitialize() 
  {
    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->getNode()->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
    tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->getNode());
  }

  std::optional<geometry_msgs::msg::PoseStamped> getGlobalPose(const std::string& child_frame_id, const std::string& frame_id)
  {
      if (tfBuffer_->canTransform(frame_id, child_frame_id, rclcpp::Time(0)))
      {
         geometry_msgs::msg::PoseStamped pose;

         auto transformStamped = tfBuffer_->lookupTransform(child_frame_id, frame_id, rclcpp::Time(0)); 
         pose.header = transformStamped.header;
         pose.pose.position.x = transformStamped.transform.translation.x;
         pose.pose.position.y = transformStamped.transform.translation.y;
         pose.pose.position.z = transformStamped.transform.translation.z;
         pose.pose.orientation = transformStamped.transform.rotation;

         return pose;
      }
      return std::nullopt;
  }

  std::optional<geometry_msgs::msg::PoseStamped> getObjectFacingPose(const std::string& child_frame_id, const std::string& frame_id)
  {
      auto globalObjectPose = this->getGlobalPose(child_frame_id, frame_id);


      // do trasnform to get the facing global pose
      if (globalObjectPose)
      {
        std::string target_facing_location = "target_facing_location";
        geometry_msgs::msg::TransformStamped facingPoseTransform;

        facingPoseTransform.header.frame_id = child_frame_id;
        facingPoseTransform.header.stamp = getNode()->now();
        facingPoseTransform.child_frame_id = target_facing_location;
        facingPoseTransform.transform.translation.x = 1.0;
        facingPoseTransform.transform.translation.y = 0.0;
        facingPoseTransform.transform.translation.z = 0.0;
        facingPoseTransform.transform.rotation.w = 1.0;
        
        //180 degree yaw - face to face to dock
        auto targetYaw = tf2::getYaw(facingPoseTransform.transform.rotation) + M_PI;
        tf2::Quaternion q;
        q.setRPY(0, 0, M_PI);

        facingPoseTransform.transform.rotation.x = q.x();
        facingPoseTransform.transform.rotation.y = q.y();
        facingPoseTransform.transform.rotation.z = q.z();
        facingPoseTransform.transform.rotation.w = q.w();

        this->tfBroadcaster_->sendTransform(facingPoseTransform);

        geometry_msgs::msg::PoseStamped facingGlobalPose;
        tf2::doTransform(*globalObjectPose, facingGlobalPose, facingPoseTransform);
        return facingGlobalPose;
      }

      return std::nullopt;
  }

  private:

    // Declare a data structure to store the detected objects.
    std::map<std::string, DetectedObject> detectedObjects;
};

} // namespace cl_apriltag_detector
