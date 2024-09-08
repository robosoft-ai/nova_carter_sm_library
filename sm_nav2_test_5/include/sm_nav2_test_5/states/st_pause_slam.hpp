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

#include <smacc2/client_behaviors/cb_ros_launch_2.hpp>
#include <smacc2/smacc.hpp>

namespace sm_nav2_test_5 {

using namespace smacc2::default_events;
using smacc2::client_behaviors::CbSleepFor;
using namespace std::chrono_literals;
using cl_nav2z::CbPauseSlam;

// STATE DECLARATION
struct StPauseSlam
    : smacc2::SmaccState<StPauseSlam, MsNav2Test1RunMode> {
  using SmaccState::SmaccState;
  // TRANSITION TABLE
  typedef mpl::list<Transition<EvCbSuccess<CbPauseSlam, OrNavigation>,
                               StFinalState, SUCCESS>
                    //  Transition<EvCbSuccess<CbSleepFor, OrNavigation>,
                    //  StStartStaticLocalization, SUCCESS>
                    >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {
    configure_orthogonal<OrNavigation, CbPauseSlam>();
    // configure_orthogonal<OrNavigation, CbSleepFor>(2s);
  }
};
} // namespace sm_nav2_test_5
