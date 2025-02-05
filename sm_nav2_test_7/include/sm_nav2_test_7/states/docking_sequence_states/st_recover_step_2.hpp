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
using cl_nav2z::CbAbsoluteRotate;
using namespace std::chrono_literals;
using namespace cl_nav2z;
using namespace cl_keyboard;

// STATE DECLARATION - Navigate to Staging
struct StRecoverStep2 : smacc2::SmaccState<StRecoverStep2, MsRecover>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct TIMEOUT : ABORT{};
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbAbsoluteRotate, OrNavigation>, StRecoverStep3, SUCCESS>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StRecoverStep3, SUCCESS>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbAbsoluteRotate>(0.0);
    // configure_orthogonal<OrNavigation, CbPauseSlam>();
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure() {}

  void onEntry() { RCLCPP_INFO(getLogger(), "On Entry!"); }

  void onExit() { RCLCPP_INFO(getLogger(), "On Exit!"); }
};
}  // namespace sm_nav2_test_7
