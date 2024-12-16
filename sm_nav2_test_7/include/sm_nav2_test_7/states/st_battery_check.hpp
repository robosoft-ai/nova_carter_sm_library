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

namespace sm_nav2_test_7 {
using namespace cl_keyboard;

// STATE DECLARATION
struct StBatteryCheck
    : smacc2::SmaccState<StBatteryCheck, MsNav2Test1RunMode> {

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
      Transition<EvWaypoint0<ClNav2Z, OrNavigation>, MsRecover, TRANSITION_1>,
      Transition<EvWaypoint1<ClNav2Z, OrNavigation>, StNavigateToWaypoint2, TRANSITION_2>,
      Transition<EvWaypoint2<ClNav2Z, OrNavigation>, MsRecover, TRANSITION_3>,
      Transition<EvWaypoint3<ClNav2Z, OrNavigation>, StNavigateToWaypoint3, TRANSITION_4>,
      Transition<EvWaypoint4<ClNav2Z, OrNavigation>, MsRecover, TRANSITION_5>,
      Transition<EvWaypoint5<ClNav2Z, OrNavigation>, StNavigateToWaypoint4, TRANSITION_6>,

      
    //  Transition<cl_nav2z::EvWaypointFinal, StNavigateToWaypoint2, SUCCESS>,
 
    //  Transition<EvCbSuccess<CbNavigateNextWaypoint, OrNavigation>, StNavigateWarehouseWaypointsX, SUCCESS>,
    //  Transition<EvCbFailure<CbNavigateNextWaypoint, OrNavigation>, StNavigateWarehouseWaypointsX, ABORT>,
    //  Transition<EvActionAborted<ClNav2Z, OrNavigation>, StNavigateWarehouseWaypointsX, ABORT>,


      //Keyboard events    
      Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StNavigateToWaypoint2, NEXT>,
      Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, StNavigateToWaypoint1, PREVIOUS>
      >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {
    // configure_orthogonal<OrNavigation, CbPositionControlFreeSpace>();
    configure_orthogonal<OrNavigation, CbNavigateNextWaypoint>();
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void onEntry() {}

  void runtimeConfigure() {}

  void onExit(ABORT) {}
};
} // namespace sm_nav2_test_7
