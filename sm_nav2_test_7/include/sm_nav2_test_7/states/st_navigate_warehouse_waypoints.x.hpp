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
struct StNavigateWarehouseWaypointsX
    : smacc2::SmaccState<StNavigateWarehouseWaypointsX,
                         MsNav2Test1RunMode> {
  using SmaccState::SmaccState;

  // CUSTOM TRANSITION TAGS
  struct TRANSITION_1 : SUCCESS {};
  struct TRANSITION_2 : SUCCESS {};
  struct TRANSITION_3 : SUCCESS {};
  struct TRANSITION_4 : SUCCESS {};
  struct TRANSITION_5 : SUCCESS {};
  struct TRANSITION_6 : SUCCESS {};
  struct TRANSITION_7 : SUCCESS {};
  struct TRANSITION_8 : SUCCESS {};
  struct TRANSITION_9 : SUCCESS {};
  struct TRANSITION_10 : SUCCESS {};
  struct TRANSITION_11 : SUCCESS {};
  struct TRANSITION_12 : SUCCESS {};
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};

  // TRANSITION TABLE
  typedef mpl::list<
      Transition<EvWaypoint0<ClNav2Z, OrNavigation>, StWaypointSpinLeft, TRANSITION_1>,
      Transition<EvWaypoint1<ClNav2Z, OrNavigation>, StWaypointSpinRight, TRANSITION_2>,
      Transition<EvWaypoint2<ClNav2Z, OrNavigation>, StWaypointSpinLeft, TRANSITION_3>,
      Transition<EvWaypoint3<ClNav2Z, OrNavigation>, StWaypointSpinRight, TRANSITION_4>,
      Transition<EvWaypoint4<ClNav2Z, OrNavigation>, StWaypointSpinLeft, TRANSITION_5>,
      Transition<EvWaypoint5<ClNav2Z, OrNavigation>, StWaypointSpinRight, TRANSITION_6>,
      Transition<EvWaypoint6<ClNav2Z, OrNavigation>, StWaypointSpinLeft, TRANSITION_7>,
      Transition<EvWaypoint7<ClNav2Z, OrNavigation>, StWaypointSpinRight, TRANSITION_8>,
      Transition<EvWaypoint8<ClNav2Z, OrNavigation>, StWaypointSpinLeft, TRANSITION_9>,
      Transition<EvWaypoint9<ClNav2Z, OrNavigation>, StWaypointSpinRight, TRANSITION_10>,
      Transition<EvWaypoint10<ClNav2Z, OrNavigation>, StWaypointSpinLeft, TRANSITION_11>,
      Transition<EvWaypoint11<ClNav2Z, OrNavigation>, StBatteryCheck, TRANSITION_12>,
      
    //  Transition<cl_nav2z::EvWaypointFinal, StNavigateToWaypoint2, SUCCESS>,
 
    //  Transition<EvCbSuccess<CbNavigateNextWaypoint, OrNavigation>, StNavigateWarehouseWaypointsX, SUCCESS>,
    //  Transition<EvCbFailure<CbNavigateNextWaypoint, OrNavigation>, StNavigateWarehouseWaypointsX, ABORT>,
    //  Transition<EvActionAborted<ClNav2Z, OrNavigation>, StNavigateWarehouseWaypointsX, ABORT>,


      //Keyboard events    
      Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StNavigateToWaypoint2, NEXT>
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
