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

#pragma once

#include <smacc2/smacc.hpp>

namespace sm_nav2_test_8 {
using namespace cl_keyboard;

// STATE DECLARATION
struct StLoadingWaypointsFile
    : smacc2::SmaccState<StLoadingWaypointsFile,
                         MsNav2Test1RunMode> {
  using SmaccState::SmaccState;

  // CUSTOM TRANSITION TAGS
  struct TRANSITION_1 : SUCCESS {};
  struct TRANSITION_2 : SUCCESS {};
  struct TRANSITION_3 : SUCCESS {};
  struct TRANSITION_4 : SUCCESS {};
  struct TRANSITION_5 : SUCCESS {};
  struct TRANSITION_6 : SUCCESS {};
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};

  // TRANSITION TABLE
  typedef mpl::list<
      Transition<EvCbSuccess<CbLoadWaypointsFile, OrNavigation>, StNavigateWarehouseWaypointsX, TRANSITION_1>,
 
      //Keyboard events    
      Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StNavigateWarehouseWaypointsX, NEXT>
      >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {
    configure_orthogonal<OrNavigation, CbLoadWaypointsFile>("waypoints_file", "sm_nav2_test_8");
  }

  void onEntry() {}

  void runtimeConfigure() {}

  void onExit(ABORT) {}
};
} // namespace sm_nav2_test_8
