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

namespace sm_nav2_test_9
{
  using cl_nav2z::CbNavigateForward;

// STATE DECLARATION - Dock
struct StRecoverStep7_b : smacc2::SmaccState<StRecoverStep7_b, MsRecover>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct TIMEOUT : ABORT{};
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};

  // TRANSITION TABLE
  typedef mpl::list<

     Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StRecoverStep8, SUCCESS>,
     Transition<EvCbSuccess<CbSleepFor, OrNavigation>, StRecoverStep8,SUCCESS>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
   // configure_orthogonal<OrTimer, CbTimerCountdownOnce>(50);
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
    // configure_orthogonal<OrNavigation, CbPauseSlam>();
    configure_orthogonal<OrNavigation, CbSleepFor>(4s);
    // configure_orthogonal<OrNavigation, CbNavigateForward>(0.5);
  }


  void runtimeConfigure() 
  {
    //this->foundationPoseRuntimeConfigure();
    // this->aprilTagBasedRuntimeConfigure();
  }

  void onEntry() { RCLCPP_INFO(getLogger(), "On Entry!"); }

  void onExit() { RCLCPP_INFO(getLogger(), "On Exit!"); }
};
}  // namespace sm_nav2_test_9
