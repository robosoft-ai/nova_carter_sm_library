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

#include <sm_nav2_test_7/clients/cl_foundationpose/client_behaviors/cb_track_object_pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


namespace sm_nav2_test_7
{
  using cl_nav2z::CbNavigateGlobalPosition;
  using cl_foundationpose::CbTrackObjectPose;

// STATE DECLARATION - Calculate Final Pose
struct StRecoverStep4 : smacc2::SmaccState<StRecoverStep4, MsRecover>, smacc2::ISmaccUpdatable
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct TIMEOUT : ABORT{};
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StRecoverStep5, SUCCESS>,
    Transition<EvCbFailure<CbNavigateGlobalPosition, OrNavigation>, StRecoverStep4, ABORT>,
    Transition<EvCbSuccess<CbNavigateGlobalPosition, OrNavigation>, StRecoverStep5, SUCCESS>
    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
    configure_orthogonal<OrNavigation, CbPauseSlam>();
  }

  void runtimeConfigure() 
  {
    CpObjectTrackerTf* objectTracker;
    requiresComponent(objectTracker);
    
    //auto pose = objectTracker->updateGlobalObjectPoseWithOffset("fp_object", "map");
    //RCLCPP_INFO(getLogger(), "[StRecoverStep3] Navigating to Facing Dock pose: %f, %f, %f", pose->pose.position.x, pose->pose.position.y, tf2::getYaw(pose->pose.orientation));
    //this->configure<OrNavigation, CbNavigateGlobalPosition>(pose->pose.position.x, pose->pose.position.y, tf2::getYaw(pose->pose.orientation));

    auto pose = objectTracker->updateAndGetGlobalPose("fp_object", "map");

    if(pose)
    {
      geometry_msgs::msg::Point dockingPoseOffset;
      dockingPoseOffset.x = -0.5;
      if(!getNode()->has_parameter("docking_pose.offset.x"))
      {
        getNode()->declare_parameter("docking_pose.offset.x",dockingPoseOffset.x);
        dockingPoseOffset.x = getNode()->get_parameter("docking_pose.offset.x").as_double();
      }

      if(!getNode()->has_parameter("docking_pose.offset.y"))
      {
        getNode()->declare_parameter("docking_pose.offset.y",dockingPoseOffset.y);
        dockingPoseOffset.y = getNode()->get_parameter("docking_pose.offset.y").as_double();
      }

      // pose->pose.position.x-=0.2;
      pose->pose.position.x+= dockingPoseOffset.x;
      pose->pose.position.y+= dockingPoseOffset.y;

      // this->configure<OrNavigation, CbNavigateGlobalPosition>(pose->pose.position.x, pose->pose.position.y, 0.0);

      // apriltag detctor
      cl_apriltag_detector::ClAprilTagDetector* apriltagDetector;
      requiresClient(apriltagDetector);

      auto tags = apriltagDetector->getTagsWithinTime(1.0s);

      //avergae apriltag position
      double avgX = 0;
      double avgY = 0;
      double avgZ = 0;
      for (auto &apriltag : tags)
      {
        avgX += apriltag.second.pose.position.x;
        avgY += apriltag.second.pose.position.y;
        avgZ += apriltag.second.pose.position.z;
      }
      avgX /= tags.size();
      avgY /= tags.size();
      avgZ /= tags.size();

      //substract some small forward offset from apriltag position for the goal position
      avgX -= 0.2;

      cl_nav2z::Pose* robotPose;
      requiresComponent(robotPose);
      auto robotPoseMsg = robotPose->toPoseStampedMsg();

      tf2::Transform robotTransform;
      tf2::fromMsg(robotPoseMsg.pose, robotTransform);

      tf2::Transform apriltagTransform;
      apriltagTransform.setOrigin(tf2::Vector3(avgX, avgY, 0.0));
      apriltagTransform.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

      globalApriltagTransform = robotTransform * apriltagTransform;

      this->configure<OrNavigation, CbNavigateGlobalPosition>(
        globalApriltagTransform.getOrigin().x(),
        globalApriltagTransform.getOrigin().y(),
        tf2::getYaw(globalApriltagTransform.getRotation())  
      );

      RCLCPP_INFO(getLogger(), "Setting target pose for docking: %f, %f", globalApriltagTransform.getOrigin().x(), globalApriltagTransform.getOrigin().y());
      markerPublisher = getNode()->create_publisher<visualization_msgs::msg::MarkerArray>("~/marker_2", 10);
    }
    else
    {
      RCLCPP_ERROR(getLogger(), "The object pose is not available. global navigation was not configured.");
    }
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPublisher;
  tf2::Transform globalApriltagTransform;

  void update() 
  {
    // publish visualization markrs of th apriltag global estimation
    visualization_msgs::msg::MarkerArray markerArray;
    
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = getNode()->now();
    marker.ns = "apriltag";
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // type sphere
    marker.type = visualization_msgs::msg::Marker::SPHERE;

    marker.pose.position.x = globalApriltagTransform.getOrigin().x();
    marker.pose.position.y = globalApriltagTransform.getOrigin().y();
    marker.pose.position.z = globalApriltagTransform.getOrigin().z();
    marker.pose.orientation.x = globalApriltagTransform.getRotation().x();

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

  void onEntry() { RCLCPP_INFO(getLogger(), "On Entry!"); }

  void onExit() { RCLCPP_INFO(getLogger(), "On Exit!"); }
};
}  // namespace sm_nav2_test_7
