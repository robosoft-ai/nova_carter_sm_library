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

namespace sm_nav2_test_7
{
    using cl_foundationpose::CbTrackObjectPose;

// STATE DECLARATION - Pause to acquire FoundationPose readings
struct StRecoverStep2 : smacc2::SmaccState<StRecoverStep2, MsRecover>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct TIMEOUT : ABORT{};
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};

  // TRANSITION TABLE
  typedef mpl::list<
     Transition<cl_foundationpose::EvObjectDetected, StRecoverStep3, SUCCESS>,
     Transition<EvCbSuccess<CbSleepFor, OrNavigation>, StRecoverStep3, SUCCESS>,
     Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StRecoverStep3, SUCCESS>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // configure_orthogonal<OrPerception, CbTrackObjectPose>("fp_object");
    configure_orthogonal<OrNavigation, CbSleepFor>(10s);
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure() {}

  void onEntry() { RCLCPP_INFO(getLogger(), "On Entry!"); }

  void onExit() { RCLCPP_INFO(getLogger(), "On Exit!"); }
};
}  // namespace sm_nav2_test_7
