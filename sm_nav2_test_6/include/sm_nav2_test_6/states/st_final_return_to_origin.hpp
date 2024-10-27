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

namespace sm_nav2_test_6 {
using namespace smacc2::default_events;
using namespace std::chrono_literals;
using cl_nav2z::CbPauseSlam;
using cl_nav2z::CbNavigateGlobalPosition;
using smacc2::client_behaviors::CbSleepFor;
using smacc2::client_behaviors::CbRosStop2;

// STATE DECLARATION
struct StFinalReturnToOrigin
    : smacc2::SmaccState<StFinalReturnToOrigin, MsNav2Test1RunMode> 
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
      Transition<EvCbSuccess<CbNavigateGlobalPosition, OrNavigation>, StSpinLeft2, SUCCESS>
      // Transition<EvCbSuccess<CbNavigateGlobalPosition, OrNavigation>, StFinalMapSaving, SUCCESS>
      // Transition<EvCbSuccess<CbNavigateGlobalPosition, OrNavigation>, SS2::SsRadialPattern2, SUCCESS>
      >
      reactions;


  // STATE FUNCTIONS
  static void staticConfigure() {
    configure_orthogonal<OrNavigation, CbNavigateGlobalPosition>(); // parameterless navigates to 0,0,0
    // configure_orthogonal<OrNavigation, CbSleepFor>(20s);
    // configure_orthogonal<OrNavigation, CbRosStop2>();
  }
};
} // namespace sm_nav2_test_6
