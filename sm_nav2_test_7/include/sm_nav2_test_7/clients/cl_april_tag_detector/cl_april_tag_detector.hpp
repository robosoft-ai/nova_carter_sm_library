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
#pragma once
#include <isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp>
#include <smacc2/smacc.hpp>
#include <smacc2/smacc_client.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace cl_apriltag_detector {
template <typename AsyncCB, typename Orthogonal>
struct EvUnvisitedAprilTagDetected
    : sc::event<EvUnvisitedAprilTagDetected<AsyncCB, Orthogonal>> {};

class ClAprilTagDetector : public smacc2::ISmaccClient {
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;

  rclcpp::Subscription<
      isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::SharedPtr
      apriltagSub_;

  smacc2::SmaccSignal<void(const isaac_ros_apriltag_interfaces::msg::
                               AprilTagDetectionArray::SharedPtr)>
      onAprilTagDetection_;

public:
  ClAprilTagDetector() {}

  void onInitialize() override {
    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->getNode()->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

    rclcpp::QoS qos(1);

    apriltagSub_ =
        this->getNode()
            ->create_subscription<
                isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
                "/tag_detections", qos,
                std::bind(&ClAprilTagDetector::onAprilTagMessageCallback, this,
                          std::placeholders::_1));
  }

  // subscribe to the apriltag detected
  template <typename T>
  boost::signals2::connection onAprilTagDetected(
      void (T::*callback)(const isaac_ros_apriltag_interfaces::msg::
                              AprilTagDetectionArray::SharedPtr &),
      T *object) {
    return this->getStateMachine()->createSignalConnection(onAprilTagDetection_,
                                                           callback, object);
  }

  virtual ~ClAprilTagDetector() {}

  std::vector<std::string> visitedWorkingAreas_;

  std::optional<std::string> selectedVisitTagId_;

  std::map<std::string, geometry_msgs::msg::PoseStamped>
      detectedAprilTagsMapPose_;

  std::mutex detectedAprilTagsMapPoseMutex_;

  std::map<std::string, geometry_msgs::msg::PoseStamped> getTagsWithinTime(rclcpp::Duration duration) {
    std::lock_guard<std::mutex> lock(detectedAprilTagsMapPoseMutex_);
    std::map<std::string, geometry_msgs::msg::PoseStamped> ret;

    auto now = getNode()->now();


    for (auto &apriltag : detectedAprilTagsMapPose_) 
    {
      rclcpp::Duration ellapsed = rclcpp::Time(apriltag.second.header.stamp) - now;

      if ( ellapsed > duration) {
        ret.insert(apriltag);
      }
    }
    return ret;
  }

private:
  void onAprilTagMessageCallback(const isaac_ros_apriltag_interfaces::msg::
                                     AprilTagDetectionArray::SharedPtr msg) {
    
    std::lock_guard<std::mutex> lock(detectedAprilTagsMapPoseMutex_);

    std::stringstream ss;
    for (auto &detection : msg->detections) {
      std::string apriltag_frameid =
          detection.family + ":" + std::to_string(detection.id);
      ss << "[ClAprilTagDetector] AprilTag detected: " << apriltag_frameid
         << std::endl;

      if (detectedAprilTagsMapPose_.find(apriltag_frameid) ==
          detectedAprilTagsMapPose_.end()) {
        // get map position using tfListener
        geometry_msgs::msg::TransformStamped transformStamped;

        try {
          // first create auxiliar transform locally z - 1 meter from the
          // detection.id frame
          geometry_msgs::msg::TransformStamped transformStampedAux;
          transformStampedAux.header.stamp = msg->header.stamp;
          transformStampedAux.header.frame_id = apriltag_frameid;
          transformStampedAux.child_frame_id = "auxiliar_" + apriltag_frameid;
          transformStampedAux.transform.translation.z = -5.0;

          // // get quaternion from yaw
          // tf2::Quaternion q;
          // q.setEuler(M_PI/2,0,0);
          // transformStamped.transform.rotation = tf2::toMsg(q);

          // add to the tfBuffer
          tfBuffer_->setTransform(transformStampedAux, "default_authority");

          transformStamped = tfBuffer_->lookupTransform(
              "map", transformStampedAux.child_frame_id, msg->header.stamp);
        } catch (tf2::TransformException &ex) {
          RCLCPP_ERROR(getLogger(), "%s", ex.what());
          return;
        }
        // transform to pose
        geometry_msgs::msg::PoseStamped poseStamped;
        poseStamped.header = transformStamped.header;
        poseStamped.pose.position.x = transformStamped.transform.translation.x;
        poseStamped.pose.position.y = transformStamped.transform.translation.y;
        poseStamped.pose.position.z = transformStamped.transform.translation.z;
        poseStamped.pose.orientation = transformStamped.transform.rotation;

        detectedAprilTagsMapPose_[apriltag_frameid] = poseStamped;

        RCLCPP_INFO_STREAM(
            getLogger(),
            "[ClAprilTagDetector] new AprilTag detected: " << apriltag_frameid);
      } else {
        RCLCPP_INFO_STREAM_THROTTLE(
            getLogger(), *(getNode()->get_clock()), 5000,
            "[ClAprilTagDetector] Skipping AprilTag already detected: "
                << apriltag_frameid);
      }
    }
    RCLCPP_INFO_STREAM_THROTTLE(getLogger(), *(getNode()->get_clock()), 5000,
                                ss.str());
    onAprilTagDetection_(msg);
  }
};

} // namespace cl_apriltag_detector
