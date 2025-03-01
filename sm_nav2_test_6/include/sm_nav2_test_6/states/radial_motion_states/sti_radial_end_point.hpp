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

namespace sm_nav2_test_6 {
namespace radial_motion_states {
using namespace cl_keyboard;

// STATE DECLARATION
struct StiRadialEndPoint : smacc2::SmaccState<StiRadialEndPoint, SS> {
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};

  // TRANSITION TABLE
  typedef mpl::list<

      Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StiRadialReturn,
                 SUCCESS>,
      Transition<EvCbFailure<CbNavigateForward, OrNavigation>, StiRadialReturn,
                 ABORT>,
     
      // Keyboard events  
      Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StiRadialReturn, NEXT>
      >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {
    // RCLCPP_INFO(getLogger(),"ssr radial end point, distance in meters: %lf",
    // SS::ray_length_meters());
    configure_orthogonal<OrNavigation, CbNavigateForward>(
        SS::ray_length_meters());
    configure_orthogonal<OrNavigation, CbPauseSlam>();
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure() {
    cl_nav2z::odom_tracker::CpOdomTracker *odomTracker;
    this->requiresComponent(odomTracker);
    auto *cbForwardMotion =
        this->template getOrthogonal<OrNavigation>()
            ->template getClientBehavior<CbNavigateForward>();
    auto previousGoal = odomTracker->getCurrentMotionGoal();

    if (previousGoal) {
      cbForwardMotion->options.forceInitialOrientation =
          previousGoal->pose.orientation;
      RCLCPP_ERROR_STREAM(this->getLogger(),
                          "Previous goal orientation: "
                              << previousGoal->pose.orientation.x << ", "
                              << previousGoal->pose.orientation.y << ", "
                              << previousGoal->pose.orientation.z << ", "
                              << previousGoal->pose.orientation.w);
    };

    RCLCPP_ERROR_STREAM(this->getLogger(), "..");
  }
};
} // namespace radial_motion_states
} // namespace sm_nav2_test_6
