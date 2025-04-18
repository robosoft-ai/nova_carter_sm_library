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

namespace sm_nav2_test_8 {
namespace f_pattern_states {
// STATE DECLARATION
template <typename SS>
struct StiFPatternForward2 : smacc2::SmaccState<StiFPatternForward2<SS>, SS> {
  typedef SmaccState<StiFPatternForward2<SS>, SS> TSti;
  using TSti::context_type;
  using TSti::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

      Transition<EvCbSuccess<CbNavigateForward, OrNavigation>,
                 StiFPatternStartLoop<SS>>,
      Transition<EvCbFailure<CbNavigateForward, OrNavigation>,
                 StiFPatternStartLoop<SS>>

      >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {
    TSti::template configure_orthogonal<OrNavigation, CbNavigateForward>(
        SS::pitch_lenght_meters());
    TSti::template configure_orthogonal<OrNavigation, CbPauseSlam>();
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
  }
};
} // namespace f_pattern_states
} // namespace sm_nav2_test_8
