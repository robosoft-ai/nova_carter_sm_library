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
// #include <nav2z_client/client_behavior>

namespace sm_nav2_test_8 {
using namespace smacc2::default_events;
using smacc2::client_behaviors::CbSleepFor;
using cl_nav2z::CbNavigateGlobalPosition;
using namespace std::chrono_literals;
using namespace cl_nav2z;
using namespace cl_keyboard;



// STATE DECLARATION
struct StNavigateToWaypoint1 : smacc2::SmaccState<StNavigateToWaypoint1, MsNav2Test1RunMode>
{
  using SmaccState::SmaccState;

   // DECLARE CUSTOM OBJECT TAGS
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbNavigateGlobalPosition, OrNavigation>, StFinalReturnToOrigin, SUCCESS>,
    //Transition<EvCbFailure<CbNavigateGlobalPosition, OrNavigation>, StNavigateWarehouseWaypointsX, ABORT>

    //Keyboard events
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StLoadingWaypointsFile, NEXT>,
    Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, StInitialMoveStop, PREVIOUS>
    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbNavigateGlobalPosition>(-9.7, 10.85, 0.0); //(-3.75, -2.25, 0.0)
    // configure_orthogonal<OrNavigation, CbResumeSlam>();
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }
};
}  // namespace sm_nav2_test_8
