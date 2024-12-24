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
using namespace smacc2::default_events;
using smacc2::client_behaviors::CbSleepFor;
using cl_nav2z::CbNavigateGlobalPosition;
using namespace std::chrono_literals;
using namespace cl_nav2z;
using namespace cl_keyboard;

// STATE DECLARATION - Navigate to Staging
struct StRecoverStep1 : smacc2::SmaccState<StRecoverStep1, MsRecover>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct TIMEOUT : ABORT{};
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbNavigateGlobalPosition, OrNavigation>, StRecoverStep2, SUCCESS>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StRecoverStep2, SUCCESS>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
   // -# configure_orthogonal<OrNavigation, CbNavigateGlobalPosition>(6.25, -13.8, 0.0);
    configure_orthogonal<OrNavigation, CbNavigateGlobalPosition>(0.0, 0.0, 0.0);
    configure_orthogonal<OrNavigation, CbResumeSlam>();
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure() 
  {

      geometry_msgs::msg::Point chargingAreaPoint;
      chargingAreaPoint.x = 0.0;
      chargingAreaPoint.y = 0.0;
      double yaw = 0.0;

      if(!getNode()->has_parameter("docking_pose.charning_area.x"))
      {
        getNode()->declare_parameter("docking_pose.charning_area.x",chargingAreaPoint.x);
        chargingAreaPoint.x = getNode()->get_parameter("docking_pose.charning_area.x").as_double();
      }

      if(!getNode()->has_parameter("docking_pose.charning_area.y"))
      {
        getNode()->declare_parameter("docking_pose.charning_area.y",chargingAreaPoint.y);
        chargingAreaPoint.y = getNode()->get_parameter("docking_pose.charning_area.y").as_double();
      }

      auto cbGlobalNavigation = this->getOrthogonal<OrNavigation>()->getClientBehavior<CbNavigateGlobalPosition>();
      cbGlobalNavigation->goalPosition = chargingAreaPoint;
      cbGlobalNavigation->goalYaw = yaw;
  }

  void onEntry() { RCLCPP_INFO(getLogger(), "On Entry!"); }

  void onExit() { RCLCPP_INFO(getLogger(), "On Exit!"); }
};
}  // namespace sm_nav2_test_7
