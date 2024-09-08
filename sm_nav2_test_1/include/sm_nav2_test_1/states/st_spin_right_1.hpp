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
#include <keyboard_client/client_behaviors/cb_default_keyboard_behavior.hpp>

namespace sm_nav2_test_1 {
using namespace smacc2::default_events;
using smacc2::client_behaviors::CbSleepFor;
using namespace std::chrono_literals;
using cl_keyboard::CbDefaultKeyboardBehavior;
using namespace cl_nav2z;

// STATE DECLARATION
struct StSpinRight1
    : smacc2::SmaccState<StSpinRight1, MsNav2Test1RunMode> {
  using SmaccState::SmaccState;

    // DECLARE CUSTOM OBJECT TAGS
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};

  // TRANSITION TABLE
  typedef mpl::list<Transition<EvCbSuccess<CbPureSpinning, OrNavigation>, StSpinLeft1, SUCCESS>,
                    Transition<EvCbFailure<CbPureSpinning, OrNavigation>, StSpinRight1, ABORT>,
                    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StSpinLeft1, NEXT>
                    >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {
    configure_orthogonal<OrNavigation, CbPureSpinning>(M_PI * 2, 0.8);
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure() {}
};
} // namespace sm_nav2_test_1
