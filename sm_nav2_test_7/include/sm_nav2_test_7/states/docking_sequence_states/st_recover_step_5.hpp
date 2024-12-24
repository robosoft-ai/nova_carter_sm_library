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

namespace sm_nav2_test_7
{
  using cl_nav2z::CbNavigateForward;

// STATE DECLARATION - Docked
struct StRecoverStep5 : smacc2::SmaccState<StRecoverStep5, MsRecover>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct TIMEOUT : ABORT{};
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};

  // TRANSITION TABLE
  typedef mpl::list<

     Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StRecoverStep6, SUCCESS>,
     Transition<EvCbSuccess<sm_nav2_test_7::CbPositionControlFreeSpace, OrNavigation>, StRecoverStep6,SUCCESS>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
   // configure_orthogonal<OrTimer, CbTimerCountdownOnce>(50);
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
    // configure_orthogonal<OrNavigation, CbNavigateForward>(0.5);
  }

  void runtimeConfigure() 
  {
    CpObjectTrackerTf* objectTracker;
    requiresComponent(objectTracker);
    auto dockingPose = objectTracker->getGlobalPose("fp_object", "map");

    cl_nav2z::Pose* robotPose;
    requiresComponent(robotPose);

    if(dockingPose)
    {
      // pose->pose.position.x-=0.05;
      // pose->pose.orientation.w = 1.0;
      // pose->pose.orientation.x = 0.0;
      // pose->pose.orientation.y = 0.0;
      // pose->pose.orientation.z = 0.0;


      auto rpose = robotPose->toPoseStampedMsg();

      bool staticForwardDistance = true;
      if(!getNode()->has_parameter("cb_battery_position_control.enable_static_forward_distance"))
      {
        getNode()->declare_parameter("cb_battery_position_control.enable_static_forward_distance",staticForwardDistance);
        staticForwardDistance = getNode()->get_parameter("cb_battery_position_control.enable_static_forward_distance").as_bool();
      }


      if(staticForwardDistance)
      {

        double forwardDistance = 0.25;
        if(!getNode()->has_parameter("cb_battery_position_control.forward_distance"))
        {
          getNode()->declare_parameter("cb_battery_position_control.forward_distance",forwardDistance);
          forwardDistance = getNode()->get_parameter("cb_battery_position_control.forward_distance").as_double();

        }
        RCLCPP_INFO(getLogger(), "Setting target pose for docking STATIC [FORWARD]: %f, %f", rpose.pose.position.x, rpose.pose.position.y);
        rpose.pose.position.x += forwardDistance;
      }
      else
      {
        double dynamicForwardOffset = 0.25;
        if(!getNode()->has_parameter("cb_battery_position_control.dynamic_forward_offset"))
        {
          getNode()->declare_parameter("cb_battery_position_control.dynamic_forward_offset",dynamicForwardOffset);
          dynamicForwardOffset = getNode()->get_parameter("cb_battery_position_control.dynamic_forward_offset").as_double();
          RCLCPP_INFO(getLogger(), "Setting dynamic forward offset: %lf", dynamicForwardOffset);

        }
        rpose.pose.position.x =  dockingPose->pose.position.x - dynamicForwardOffset;
        RCLCPP_INFO(getLogger(), "Setting target pose for docking DYNAMIC: %f, %f", rpose.pose.position.x, rpose.pose.position.y);
      }

      auto cb_position_control=this->configure<OrNavigation, sm_nav2_test_7::CbPositionControlFreeSpace>();
      cb_position_control->target_pose_ = rpose.pose;

      cb_position_control->max_linear_velocity = 0.1;
      if(!getNode()->has_parameter("cb_battery_position_control.max_linear_velocity"))
      {
        getNode()->declare_parameter("cb_battery_position_control.max_linear_velocity",cb_position_control->max_linear_velocity);
        cb_position_control->max_linear_velocity = getNode()->get_parameter("cb_battery_position_control.max_linear_velocity").as_double();
      }

      cb_position_control->threshold_distance_ = 0.05;
      if(!getNode()->has_parameter("cb_battery_position_control.threshold_distance"))
      {
        getNode()->declare_parameter("cb_battery_position_control.threshold_distance",cb_position_control->threshold_distance_);
        cb_position_control->threshold_distance_ = getNode()->get_parameter("cb_battery_position_control.threshold_distance").as_double();
      }

      cb_position_control->kp_angular = 0.0; // 0.5
      if (!getNode()->has_parameter("cb_battery_position_control.kp_angular"))
      {
        getNode()->declare_parameter("cb_battery_position_control.kp_angular", cb_position_control->kp_angular);
        cb_position_control->kp_angular = getNode()->get_parameter("cb_battery_position_control.kp_angular").as_double();
      }

      cb_position_control->kd_angular=0.0;
      if (!getNode()->has_parameter("cb_battery_position_control.kd_angular"))
      {
        getNode()->declare_parameter("cb_battery_position_control.kd_angular", cb_position_control->kd_angular);
        cb_position_control->kd_angular = getNode()->get_parameter("cb_battery_position_control.kd_angular").as_double();
      }

      cb_position_control->command_timeout_sec = 10.0;
      if (!getNode()->has_parameter("cb_battery_position_control.command_timeout_sec"))
      {
        getNode()->declare_parameter("cb_battery_position_control.command_timeout_sec", cb_position_control->command_timeout_sec);
        cb_position_control->command_timeout_sec = getNode()->get_parameter("cb_battery_position_control.command_timeout_sec").as_double();
      }
      
      RCLCPP_INFO(getLogger(), "Setting target pose for docking: %f, %f", rpose.pose.position.x, rpose.pose.position.y);
      RCLCPP_INFO(getLogger(), "Setting max linear velocity: %lf", cb_position_control->max_linear_velocity);
      RCLCPP_INFO(getLogger(), "Setting threshold distance: %lf", cb_position_control->threshold_distance_);
      RCLCPP_INFO(getLogger(), "Setting kp angular: %lf", cb_position_control->kp_angular);
      RCLCPP_INFO(getLogger(), "Setting kd angular: %lf", cb_position_control->kd_angular);

    }
    else
    {
      RCLCPP_ERROR(getLogger(), "The object pose is not available. global navigation was not configured.");
    }
  }

  void onEntry() { RCLCPP_INFO(getLogger(), "On Entry!"); }

  void onExit() { RCLCPP_INFO(getLogger(), "On Exit!"); }
};
}  // namespace sm_nav2_test_7
