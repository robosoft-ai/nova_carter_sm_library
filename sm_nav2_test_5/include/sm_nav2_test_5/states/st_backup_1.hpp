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

namespace sm_nav2_test_5 {
using namespace cl_nav2z;  
using namespace cl_keyboard;

// STATE DECLARATION
struct StBackup1 : smacc2::SmaccState<StBackup1, MsNav2Test1RunMode> {
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};

  // TRANSITION TABLE
  typedef mpl::list<

      Transition<EvCbSuccess<CbNavigateBackwards, OrNavigation>, StFinalState,
                 SUCCESS>,
      Transition<EvCbFailure<CbNavigateBackwards, OrNavigation>, StFinalState,
                 ABORT>,
     
      // Keyboard events  
      Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StFinalState, NEXT>
      >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {
    // RCLCPP_INFO(getLogger(),"ssr radial end point, distance in meters: %lf",
    // SS::ray_length_meters());
    configure_orthogonal<OrNavigation, CbNavigateBackwards>(3.0);
    //configure_orthogonal<OrNavigation, CbPauseSlam>();
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure() {}
};
} // namespace sm_nav2_test_5
