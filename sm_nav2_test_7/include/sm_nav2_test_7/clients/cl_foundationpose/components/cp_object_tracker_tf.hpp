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
#include <smacc2/smacc_updatable.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <sm_nav2_test_7/clients/cl_foundationpose/components/tracker_utils.hpp>

namespace cl_foundationpose 
{

using namespace smacc2::components;


class CpObjectTrackerTf : public smacc2::ISmaccComponent , public smacc2::ISmaccUpdatable
{

private:
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
  std::string global_frame_id_;
  bool enabled_= false;

public:

CpObjectTrackerTf(std::string global_frame_id="map") : global_frame_id_(global_frame_id) {}

  void setEnabled(bool enabled) 
  {
    enabled_ = enabled;
  }

  void update() override
  {
    if(!enabled_)
      return;
    
    //throttle
    //RCLCPP_INFO(getLogger(), "CpObjectTrackerTf::update() heartbeat");
    RCLCPP_INFO_THROTTLE(getLogger(), *getNode(), 5000, "CpObjectTrackerTf::update() heartbeat, tracked objects: %ld", detectedObjects.size());

    // refresh tracked object poses
    for (auto &detectedObject : detectedObjects)
    {
      RCLCPP_INFO_THROTTLE(getLogger(), *getNode(), 5000, "CpObjectTrackerTf::update() tracking object: %s", detectedObject.first.c_str());
      
      auto globalObjectPose = this->getGlobalPose(detectedObject.first, global_frame_id_);
      if (globalObjectPose)
      {
        
        detectedObject.second.filtered_pose = *globalObjectPose;

        this->updateGlobalObjectPoseWithOffset(detectedObject.first, global_frame_id_);
      }
    }
  }


  void onInitialize() 
  {
    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->getNode()->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
    tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->getNode());
  }

  std::optional<geometry_msgs::msg::PoseStamped> getGlobalPose(const std::string& child_frame_id, const std::string& frame_id)
  {
    RCLCPP_INFO(getLogger(), "[CpObjectTrackerTf] getGlobalPose('%s', '%s')", child_frame_id.c_str(), frame_id.c_str());
    // check in database if the object is already tracked
    auto object = detectedObjects.find(child_frame_id);
    DetectedObject* detectedObject = nullptr;
    if (object == detectedObjects.end()) // already tracked
    {
      detectedObject = &object->second;
    }
    else
    {
      detectedObjects[child_frame_id] = DetectedObject();
      detectedObject = &detectedObjects[child_frame_id];
      RCLCPP_INFO(getLogger(), "[CpObjectTrackerTf] tracking new object: %s", child_frame_id.c_str());
    }

    if (tfBuffer_->canTransform(frame_id, child_frame_id, rclcpp::Time(0)))
    {
         geometry_msgs::msg::PoseStamped pose;

         auto transformStamped = tfBuffer_->lookupTransform(frame_id, child_frame_id, rclcpp::Time(0)); 
         pose.header = transformStamped.header;
         pose.pose.position.x = transformStamped.transform.translation.x;
         pose.pose.position.y = transformStamped.transform.translation.y;
         pose.pose.position.z = transformStamped.transform.translation.z;
         pose.pose.orientation = transformStamped.transform.rotation;

         RCLCPP_INFO(getLogger(), "[CpObjectTrackerTf] getGlobalPose('%s', '%s') pose: %f, %f, %f", child_frame_id.c_str(), frame_id.c_str(), pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation));

         if (!detectedObject->filtered_pose)
         {
            detectedObject->filtered_pose = pose;
         }

         detectedObject->filtered_pose = pose;
         this->postEvent<EvObjectDetected>();


        //  //second
        //  auto t2 = tfBuffer_->lookupTransform(frame_id, child_frame_id ,rclcpp::Time(0));
        //  RCLCPP_INFO(getLogger(), "[CpObjectTrackerTf] getGlobalPose('%s', '%s') inverted pose: %f, %f, %f", frame_id.c_str(), child_frame_id.c_str(), t2.transform.translation.x, t2.transform.translation.y, tf2::getYaw(t2.transform.rotation));

         return pose;
    }

    return std::nullopt;
  }

  std::optional<geometry_msgs::msg::PoseStamped> updateGlobalObjectPoseWithOffset(const std::string& child_frame_id, const std::string& frame_id)
  {
      auto globalObjectPose = this->getGlobalPose(child_frame_id, frame_id);
      RCLCPP_INFO(getLogger(), "[CpObjectTrackerTf] globalObjectPose[%s]: %f, %f, %f", global_frame_id_.c_str(), globalObjectPose->pose.position.x, globalObjectPose->pose.position.y, tf2::getYaw(globalObjectPose->pose.orientation));

      std::string target_facing_location = "target_facing_location";

      // do trasnform to get the facing global pose
      if (globalObjectPose)
      {
        RCLCPP_INFO(getLogger(), "[CpObjectTrackerTf] local facing pose transform requested");
        geometry_msgs::msg::TransformStamped dockingStationOffsetTransform;

        dockingStationOffsetTransform.header.frame_id = child_frame_id;
        dockingStationOffsetTransform.header.stamp = getNode()->now();
        dockingStationOffsetTransform.child_frame_id = target_facing_location;
        dockingStationOffsetTransform.transform.translation.x = 0.5;
        dockingStationOffsetTransform.transform.translation.y = 0.0;
        dockingStationOffsetTransform.transform.translation.z = 0.0;
        dockingStationOffsetTransform.transform.rotation.w = 1.0;
        

        this->tfBroadcaster_->sendTransform(dockingStationOffsetTransform);

        RCLCPP_INFO(getLogger(), "[CpObjectTrackerTf] local facing pose transform [%s] sent: %f, %f, %f",dockingStationOffsetTransform.header.frame_id.c_str(), dockingStationOffsetTransform.transform.translation.x, dockingStationOffsetTransform.transform.translation.y, tf2::getYaw(dockingStationOffsetTransform.transform.rotation));
        

        //------------------------------------

        geometry_msgs::msg::Pose initialRotation;
        tf2::Quaternion q;
        q.setRPY(0, 0, M_PI);

        initialRotation.orientation.x = q.x();
        initialRotation.orientation.y = q.y();
        initialRotation.orientation.z = q.z();
        initialRotation.orientation.w = q.w();

        geometry_msgs::msg::Pose dockingOffsetWithOrientation;
        tf2::doTransform(initialRotation, dockingOffsetWithOrientation, dockingStationOffsetTransform);

        RCLCPP_INFO(getLogger(), "[CpObjectTrackerTf] dockingOffsetWithOrientation[%s]: %f, %f, %f",global_frame_id_.c_str(), dockingOffsetWithOrientation.position.x, dockingOffsetWithOrientation.position.y, tf2::getYaw(dockingOffsetWithOrientation.orientation));

        //------------------------------------
        geometry_msgs::msg::TransformStamped globalDockingPositionTransform;

        globalDockingPositionTransform.transform.translation.x = globalObjectPose->pose.position.x;
        globalDockingPositionTransform.transform.translation.y = globalObjectPose->pose.position.y;
        globalDockingPositionTransform.transform.translation.z = globalObjectPose->pose.position.z;
        globalDockingPositionTransform.transform.rotation.w = globalObjectPose->pose.orientation.w;
        globalDockingPositionTransform.transform.rotation.x = globalObjectPose->pose.orientation.x;
        globalDockingPositionTransform.transform.rotation.y = globalObjectPose->pose.orientation.y;
        globalDockingPositionTransform.transform.rotation.z = globalObjectPose->pose.orientation.z;


        geometry_msgs::msg::Pose globalDockingPositionWithOffset;
        tf2::doTransform(dockingOffsetWithOrientation, globalDockingPositionWithOffset, globalDockingPositionTransform);

        // RCLCPP_INFO(getLogger(), "[CpObjectTrackerTf] globalDockingPositionWithOffset[%s]: %f, %f, %f",global_frame_id_.c_str(), globalDockingPositionWithOffset.pose.position.x, globalDockingPositionWithOffset.pose.position.y, tf2::getYaw(globalDockingPositionWithOffset.pose.orientation));
        RCLCPP_INFO(getLogger(), "[CpObjectTrackerTf] globalDockingPositionWithOffset[%s]: %f, %f, %f",global_frame_id_.c_str(), globalDockingPositionWithOffset.position.x, globalDockingPositionWithOffset.position.y, tf2::getYaw(globalDockingPositionWithOffset.orientation));
        //------------------------------------


        geometry_msgs::msg::TransformStamped finalGlobalTargetPosition;

        finalGlobalTargetPosition.header.frame_id = global_frame_id_;
        finalGlobalTargetPosition.header.stamp = getNode()->now();
        finalGlobalTargetPosition.child_frame_id = std::string("global_")+target_facing_location;
        finalGlobalTargetPosition.transform.translation.x = globalDockingPositionWithOffset.position.x;
        finalGlobalTargetPosition.transform.translation.y = globalDockingPositionWithOffset.position.y;
        finalGlobalTargetPosition.transform.translation.z = globalDockingPositionWithOffset.position.z;
        finalGlobalTargetPosition.transform.rotation.w = globalDockingPositionWithOffset.orientation.w;
        finalGlobalTargetPosition.transform.rotation.x = globalDockingPositionWithOffset.orientation.x;
        finalGlobalTargetPosition.transform.rotation.y = globalDockingPositionWithOffset.orientation.y;
        finalGlobalTargetPosition.transform.rotation.z = globalDockingPositionWithOffset.orientation.z;

        this->tfBroadcaster_->sendTransform(finalGlobalTargetPosition);
                

        geometry_msgs::msg::PoseStamped facingGlobalPose;
        facingGlobalPose.header.frame_id = global_frame_id_;
        facingGlobalPose.header.stamp = getNode()->now();
        facingGlobalPose.pose.position.x = finalGlobalTargetPosition.transform.translation.x;
        facingGlobalPose.pose.position.y = finalGlobalTargetPosition.transform.translation.y;
        facingGlobalPose.pose.position.z = finalGlobalTargetPosition.transform.translation.z;
        facingGlobalPose.pose.orientation.w = finalGlobalTargetPosition.transform.rotation.w;
        facingGlobalPose.pose.orientation.x = finalGlobalTargetPosition.transform.rotation.x;
        facingGlobalPose.pose.orientation.y = finalGlobalTargetPosition.transform.rotation.y;
        facingGlobalPose.pose.orientation.z = finalGlobalTargetPosition.transform.rotation.z;
        return facingGlobalPose;
      }

      // if(tfBuffer_->canTransform(target_facing_location, global_frame_id_, rclcpp::Time(0)))
      // {
      //   RCLCPP_INFO(getLogger(), "[CpObjectTrackerTf] facing global pose transform requested");
      //   auto transformStamped = tfBuffer_->lookupTransform(target_facing_location, global_frame_id_ , rclcpp::Time(0)); 
      //   geometry_msgs::msg::PoseStamped pose;
      //   pose.header = transformStamped.header;
      //   pose.pose.position.x = transformStamped.transform.translation.x;
      //   pose.pose.position.y = transformStamped.transform.translation.y;
      //   pose.pose.position.z = transformStamped.transform.translation.z;
      //   pose.pose.orientation = transformStamped.transform.rotation;

      //   RCLCPP_INFO(getLogger(), "[CpObjectTrackerTf] facing global pose: %f, %f, %f", pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation));
      //   return pose;
      // }

      RCLCPP_INFO(getLogger(), "[CpObjectTrackerTf] facing global pose not found");       

      return std::nullopt;
  }

  private:
    // Declare a data structure to store the detected objects.
    std::map<std::string, DetectedObject> detectedObjects;
};

} // namespace cl_apriltag_detector
