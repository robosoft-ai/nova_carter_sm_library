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

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace sm_nav2_test_7
{
  using cl_nav2z::CbNavigateForward;

// STATE DECLARATION - Dock
struct StRecoverStep7_a : smacc2::SmaccState<StRecoverStep7_a, MsRecover>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct TIMEOUT : ABORT{};
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};

  // TRANSITION TABLE
  typedef mpl::list<

     Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StRecoverStep7_b, SUCCESS>,
     Transition<EvCbSuccess<sm_nav2_test_7::CbPositionControlFreeSpace, OrNavigation>, StRecoverStep7_b,SUCCESS>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
   // configure_orthogonal<OrTimer, CbTimerCountdownOnce>(50);
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
    configure_orthogonal<OrNavigation, CbPauseSlam>();
    // configure_orthogonal<OrNavigation, CbNavigateForward>(0.5);
  }

  void foundationPoseRuntimeConfigure() 
  {
    CpObjectTrackerTf* objectTracker;
    requiresComponent(objectTracker);
    auto dockingPose = objectTracker->updateAndGetGlobalPose("fp_object", "map");

    cl_nav2z::Pose* robotPose;
    requiresComponent(robotPose);

    if(dockingPose)
    {
      auto robotposemsg = robotPose->toPoseStampedMsg();
      auto motion_target_pose = decorateTargetPoseFromParameters( robotposemsg.pose, dockingPose->pose);

      RCLCPP_INFO(getLogger(), "Setting target pose for docking: %f, %f", motion_target_pose.position.x, motion_target_pose.position.y);
      auto cb_position_control=this->configure<OrNavigation, sm_nav2_test_7::CbPositionControlFreeSpace>();
      // cb_position_control->target_pose_ = motion_target_pose.pose;

      cb_position_control->getTargetPoseCallback_= [motion_target_pose](){return std::optional<geometry_msgs::msg::Pose>(motion_target_pose);};
    }
    else
    {
      RCLCPP_ERROR(getLogger(), "The object pose is not available. global navigation was not configured.");
    }
  }

  void aprilTagBasedRuntimeConfigure()
  {
      cl_nav2z::Pose* robotPose;
      requiresComponent(robotPose);

      cl_apriltag_detector::ClAprilTagDetector* apriltagDetector;
      requiresClient(apriltagDetector);

      cl_apriltag_detector::CpAprilTagVisualization* markerPublisher = apriltagDetector->getComponent<cl_apriltag_detector::CpAprilTagVisualization>();

      markerPublisher->computeAggregatdMarker();
      auto globalApriltagTransform = markerPublisher->getGlobalApriltagTransform();

      RCLCPP_INFO(getLogger(), "[StRecoverStep7] Global AprilTag Transform: %s", globalApriltagTransform ? "FOUND" : "NOT FOUND");

      if(globalApriltagTransform)
      {
      
        auto cb_position_control=this->configure<OrNavigation, sm_nav2_test_7::CbPositionControlFreeSpace>();

        cb_position_control->getTargetPoseCallback_= [this, robotPose, markerPublisher]()
        {
            auto robotposemsg = robotPose->toPoseStampedMsg();

            geometry_msgs::msg::Pose dockingPose;

            //toPose tf2

            auto globalApriltagTransform = markerPublisher->getGlobalApriltagTransform();

            if(!globalApriltagTransform)
              return std::optional<geometry_msgs::msg::Pose>(); //std::nullopt;

            RCLCPP_INFO(getLogger(), "Robot pose: %f, %f", robotposemsg.pose.position.x, robotposemsg.pose.position.y);
            RCLCPP_INFO(getLogger(), "Apriltag pose: %f, %f", globalApriltagTransform->getOrigin().x(), globalApriltagTransform->getOrigin().y());

            // dockingPose = tf2::toPose(*globalApriltagTransform);

            dockingPose.position.x = globalApriltagTransform->getOrigin().x();
            dockingPose.position.y = globalApriltagTransform->getOrigin().y();
            dockingPose.position.z = globalApriltagTransform->getOrigin().z();

            dockingPose.orientation.x = globalApriltagTransform->getRotation().x();
            dockingPose.orientation.y = globalApriltagTransform->getRotation().y();
            dockingPose.orientation.z = globalApriltagTransform->getRotation().z();
            dockingPose.orientation.w = globalApriltagTransform->getRotation().w();

            // auto motion_target_pose = decorateTargetPoseFromParameters( robotposemsg.pose, dockingPose);
            auto motion_target_pose = dockingPose;

            RCLCPP_INFO(getLogger(), "Setting target pose for docking: %f, %f", motion_target_pose.position.x, motion_target_pose.position.y);
            
            return std::optional<geometry_msgs::msg::Pose>(motion_target_pose); //motion_target_pose;
        };
      }
      else
      {
        RCLCPP_ERROR(getLogger(), "The docking station pose is not available. global navigation was not configured.");
      }
  }

  geometry_msgs::msg::Pose decorateTargetPoseFromParameters(const geometry_msgs::msg::Pose& robotPose, const geometry_msgs::msg::Pose& dockingPose)
  {
      geometry_msgs::msg::Pose motion_target_pose = robotPose;
      bool staticForwardDistance = true;
      if(!getNode()->has_parameter("cb_battery_position_control.enable_static_forward_distance"))
      {
        getNode()->declare_parameter("cb_battery_position_control.enable_static_forward_distance",staticForwardDistance);
        staticForwardDistance = getNode()->get_parameter("cb_battery_position_control.enable_static_forward_distance").as_bool();
      }

      if(staticForwardDistance)
      {
        // the robot will move forward some static/fixed distance
        double forwardDistance = 0.25;
        if(!getNode()->has_parameter("cb_battery_position_control.forward_distance"))
        {
          getNode()->declare_parameter("cb_battery_position_control.forward_distance",forwardDistance);
          forwardDistance = getNode()->get_parameter("cb_battery_position_control.forward_distance").as_double();

        }
        RCLCPP_INFO(getLogger(), "Setting target pose for docking STATIC [FORWARD]: %f, %f", motion_target_pose.position.x, motion_target_pose.position.y);
        motion_target_pose.position.x += forwardDistance;
      }
      else
      {
        // the robot will move forward some dynamic distance based on the detected distance to the docking station
        double dynamicForwardOffset = 0.25;
        if(!getNode()->has_parameter("cb_battery_position_control.dynamic_forward_offset"))
        {
          getNode()->declare_parameter("cb_battery_position_control.dynamic_forward_offset",dynamicForwardOffset);
          dynamicForwardOffset = getNode()->get_parameter("cb_battery_position_control.dynamic_forward_offset").as_double();
          RCLCPP_INFO(getLogger(), "Setting dynamic forward offset: %lf", dynamicForwardOffset);

        }
        motion_target_pose.position.x =  dockingPose.position.x - dynamicForwardOffset;
        RCLCPP_INFO(getLogger(), "Setting target pose for docking DYNAMIC: %f, %f", motion_target_pose.position.x, motion_target_pose.position.y);
      }

      return motion_target_pose;
  }

  void runtimeConfigure() 
  {
    //this->foundationPoseRuntimeConfigure();
    this->aprilTagBasedRuntimeConfigure();
  }

  void onEntry() { RCLCPP_INFO(getLogger(), "On Entry!"); }

  void onExit() { RCLCPP_INFO(getLogger(), "On Exit!"); }
};
}  // namespace sm_nav2_test_7
