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

namespace sm_nav2_test_7 {
namespace radial_motion_states {
using namespace cl_keyboard;

// STATE DECLARATION
struct StiRadialReturn : smacc2::SmaccState<StiRadialReturn, SS> {
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};

  // TRANSITION TABLE
  typedef mpl::list<

      Transition<EvCbSuccess<CbUndoPathBackwards, OrNavigation>,
                 StiRadialLoopStart, SUCCESS>,
      // Transition<EvCbFailure<CbUndoPathBackwards, OrNavigation>,
      //            StiRadialEndPoint, ABORT>,

      // Keyboard events  
      Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StiRadialLoopStart, NEXT>
      >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {
    // configure_orthogonal<OrNavigation, CbUndoPathBackwards>();

    // Create serializable struct for parameterization options - see CbUndoPathBackwards.hpp
    cl_nav2z::CbUndoPathBackwardsOptions options;
    // Select the specific goal checker to use - see config/nav2_config.yaml
    options.goalCheckerId_ = "undo_path_backwards_goal_checker_2";
    
    configure_orthogonal<OrNavigation, CbUndoPathBackwards>(options);

    configure_orthogonal<OrNavigation, CbPauseSlam>();
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void onExit() {
    ClNav2Z *moveBase;
    this->requiresClient(moveBase);

    auto odomTracker =
        moveBase->getComponent<cl_nav2z::odom_tracker::CpOdomTracker>();
    odomTracker->clearPath();
  }
};
} // namespace radial_motion_states
} // namespace sm_nav2_test_7
