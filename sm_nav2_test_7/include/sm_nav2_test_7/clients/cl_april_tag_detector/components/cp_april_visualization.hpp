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

#include <smacc2/common.hpp>
#include <smacc2/component.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <mutex>
#include <vector>

#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;
using std::chrono_literals::operator""s;

namespace cl_apriltag_detector
{

class CpAprilTagVisualization : public smacc2::ISmaccComponent, public smacc2::ISmaccUpdatable
{
public:
  CpAprilTagVisualization()
  {

  }

  virtual ~CpAprilTagVisualization()
  {
  }

  void onInitialize() override
  {
      markerPublisher = getNode()->create_publisher<visualization_msgs::msg::MarkerArray>("/markers_2", 10);
    
  }

  std::optional<tf2::Transform> getGlobalApriltagTransform()
  {
    std::lock_guard<std::mutex> lock(m_mutex_);
    return globalApriltagTransform;
  }

  void computeAggregatdMarker()
  {
    RCLCPP_INFO(getLogger(), "[CpAprilTagVisualization] computeAggregatdMarker");
     cl_apriltag_detector::ClAprilTagDetector* apriltagDetector = dynamic_cast<cl_apriltag_detector::ClAprilTagDetector*>(this->owner_);
      
      RCLCPP_INFO(getLogger(), "[CpAprilTagVisualization] getTagsWithinTime");
      auto tags = apriltagDetector->getTagsWithinTime(0.5s);

      //avergae apriltag position
      double avgX = 0;
      double avgY = 0;
      double avgZ = 0;
      for (auto &apriltag : tags)
      {
        avgX += apriltag.second.pose.position.x;
        avgY += apriltag.second.pose.position.y;
        avgZ += apriltag.second.pose.position.z;

        //RCLCPP_INFO(getLogger(), "[CpAprilTagVisualization] apriltag: %f, %f, %f", apriltag.second.pose.position.x, apriltag.second.pose.position.y, apriltag.second.pose.position.z);
      }

      if(tags.size() == 0)
      {
        RCLCPP_INFO_THROTTLE(getLogger(), *(getNode()->get_clock()), 1000, "[CpAprilTagVisualization] no apriltags detected to visualize");
        return;
      }

      avgX /= tags.size();
      avgY /= tags.size();
      avgZ /= tags.size();

      //substract some small forward offset from apriltag position for the goal position
      avgX -= 0.15;

      RCLCPP_INFO(getLogger(), "[CpAprilTagVisualization] detected %ld tags", tags.size());
      RCLCPP_INFO(getLogger(), "[CpAprilTagVisualization] avgX: %f, avgY: %f, avgZ: %f", avgX, avgY, avgZ);
      // cl_nav2z::Pose* robotPose;
      // RCLCPP_INFO(getLogger(), "[CpAprilTagVisualization] robotPose");
      // // requiresComponent(robotPose, true);

      // this->getStateMachine()->requiresComponent(robotPose);
      // RCLCPP_INFO(getLogger(), "[CpAprilTagVisualization] robotPoseMsg");
      // auto robotPoseMsg = robotPose->toPoseStampedMsg();

      // tf2::Transform robotTransform;
      // tf2::fromMsg(robotPoseMsg.pose, robotTransform);

      // RCLCPP_INFO(getLogger(), "[CpAprilTagVisualization] robotTransform: %f, %f, %f", robotTransform.getOrigin().x(), robotTransform.getOrigin().y(), robotTransform.getOrigin().z());
      tf2::Transform apriltagTransform;
      apriltagTransform.setOrigin(tf2::Vector3(avgX, avgY, 0.0));
      apriltagTransform.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

      // globalApriltagTransform = robotTransform * apriltagTransform;

      globalApriltagTransform = apriltagTransform;

      // robot posee
      // RCLCPP_INFO(getLogger(), "[CpAprilTagVisualization] robotTransform: %f, %f, %f", robotTransform.getOrigin().x(), robotTransform.getOrigin().y(), robotTransform.getOrigin().z());

      //local apriltag transform
      RCLCPP_INFO(getLogger(), "[CpAprilTagVisualization] apriltagTransform: %f, %f, %f", apriltagTransform.getOrigin().x(), apriltagTransform.getOrigin().y(), apriltagTransform.getOrigin().z());

      //global apriltag transform
      RCLCPP_INFO(getLogger(), "[CpAprilTagVisualization] globalApriltagTransform: %f, %f, %f", globalApriltagTransform->getOrigin().x(), globalApriltagTransform->getOrigin().y(), globalApriltagTransform->getOrigin().z());
  }

  virtual void update() override
  {

    if (globalApriltagTransform)
    {

      this->computeAggregatdMarker();
        
      // publish visualization markrs of th apriltag global estimation
      visualization_msgs::msg::MarkerArray markerArray;
      
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = getNode()->now();
      marker.ns = "apriltag";
      marker.id = 0;
      marker.action = visualization_msgs::msg::Marker::ADD;

      //lifetim of 0.5s
      marker.lifetime = rclcpp::Duration(0.5s);

      // type sphere
      marker.type = visualization_msgs::msg::Marker::SPHERE;

      auto origin = globalApriltagTransform->getOrigin();
      marker.pose.position.x = origin.x();
      marker.pose.position.y = origin.y();
      marker.pose.position.z = origin.z();

      auto rotation = globalApriltagTransform->getRotation();
      marker.pose.orientation.w = rotation.w();
      marker.pose.orientation.y = rotation.y();
      marker.pose.orientation.z = rotation.z();
      marker.pose.orientation.x = rotation.x();
      
      RCLCPP_INFO_THROTTLE(getLogger(), *(getNode()->get_clock()), 1000, "[CpAprilTagVisualization] marker position: %f, %f, %f", origin.x(), origin.y(), origin.z());
      
      // red color
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;

      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;

      markerArray.markers.push_back(marker);

      markerPublisher->publish(markerArray);
  }
  }

protected:

  std::mutex m_mutex_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPublisher;
  std::optional<tf2::Transform> globalApriltagTransform;
};

}  // namespace cp_april_tag
