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
namespace sm_nav2_test_6
{
using namespace cl_nav2z;
using namespace cl_keyboard;

// STATE DECLARATION
struct StWaypointSpinRight : smacc2::SmaccState<StWaypointSpinRight, MsNav2Test1RunMode>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbPureSpinning, OrNavigation>, StNavigateWarehouseWaypointsX, SUCCESS>,
    Transition<EvCbFailure<CbPureSpinning, OrNavigation>, StNavigateWarehouseWaypointsX, ABORT>,

    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StNavigateWarehouseWaypointsX, NEXT>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbPureSpinning>(2*M_PI, 1.0 /*rad_s*/);
    //configure_orthogonal<OrNavigation, CbResumeSlam>();
  }
};
}  // namespace sm_nav2_test_6
