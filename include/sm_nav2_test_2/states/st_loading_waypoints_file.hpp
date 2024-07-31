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

namespace sm_nav2_test_2 {
using namespace cl_keyboard;

// STATE DECLARATION
struct StLoadingWayPointsFile
    : smacc2::SmaccState<StLoadingWayPointsFile,
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
      // cl_nav2z::EvWaypointFinal, StFinalState, SUCCESS>,
      // Transition<EvWaypoint3<ClNav2Z, OrNavigation>, SS5::SsSPattern1,
      // SUCCESS>, Transition<EvWaypoint11<ClNav2Z, OrNavigation>,
      // SS4::SsFPattern1, SUCCESS>, Transition<EvWaypoint1<ClNav2Z,
      // OrNavigation>, SS1::SsRadialPattern1, SUCCESS>,
      // Transition<EvCbSuccess<CbNavigateNextWaypoint, OrNavigation>, StNavigateWarehouseWaypointsX, SUCCESS>,
      // Transition<EvCbFailure<CbNavigateNextWaypoint, OrNavigation>, StNavigateWarehouseWaypointsX, ABORT>,
      // Transition<EvActionAborted<ClNav2Z, OrNavigation>, StNavigateWarehouseWaypointsX, ABORT>,
      // Transition<EvWaypoint2<ClNav2Z, OrNavigation>, SS2::SsRadialPattern2,
      // TRANSITION_4> Transition<EvWaypoint5<ClNav2Z, OrNavigation>,
      // StStartStaticLocalization, SUCCESS>

      
      //Keyboard events    
      Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StNavigateWarehouseWaypointsX, NEXT>,
      Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, StNavigateToWaypoint1, PREVIOUS>
      >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {
    // configure_orthogonal<OrNavigation, CbPositionControlFreeSpace>();
    configure_orthogonal<OrNavigation, CbLoadWaypointsFile>("waypoints_file", "sm_nav2_test_2");
  }

  void onEntry() {}

  void runtimeConfigure() {}

  void onExit(ABORT) {}
};
} // namespace sm_nav2_test_2
