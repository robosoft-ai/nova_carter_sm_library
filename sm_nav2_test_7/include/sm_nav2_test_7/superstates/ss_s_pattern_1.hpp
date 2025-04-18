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

#include <smacc2/smacc.hpp>

namespace sm_nav2_test_7 {
namespace SS2 {
namespace sm_nav2_test_7 {
namespace s_pattern_states {
// FORWARD DECLARATIONS OF INNER STATES
class StiSPatternRotate1;
class StiSPatternForward1;
class StiSPatternRotate2;
class StiSPatternForward2;
class StiSPatternRotate3;
class StiSPatternForward3;
class StiSPatternRotate4;
class StiSPatternForward4;
class StiSPatternLoopStart;
} // namespace s_pattern_states
} // namespace sm_nav2_test_7

enum class TDirection { LEFT, RIGHT };

using namespace sm_nav2_test_7::s_pattern_states;

// STATE DECLARATION
struct SsSPattern1 : smacc2::SmaccState<SsSPattern1, MsNav2Test1RunMode,
                                        StiSPatternLoopStart> {
public:
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

      // Transition<EvLoopEnd<StiSPatternLoopStart>, StNavigateReverse3,
      // ENDLOOP>
      Transition<EvLoopEnd<StiSPatternLoopStart>, StBatteryCheck,
                 ENDLOOP> //,StNavigateToWaypoint4

      >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {}

  static constexpr float pitch1_lenght_meters() { return 0.5; }
  static constexpr float pitch2_lenght_meters() { return 27.0; }
  static constexpr int total_iterations() { return 9; }
  static constexpr TDirection direction() { return TDirection::RIGHT; }

  int iteration_count;

  void runtimeConfigure() { this->iteration_count = 0; }
};

// FORWARD DECLARATION FOR THE SUPERSTATE
using SS = SsSPattern1;
#include <sm_nav2_test_7/states/s_pattern_states/sti_spattern_forward_1.hpp>
#include <sm_nav2_test_7/states/s_pattern_states/sti_spattern_forward_2.hpp>
#include <sm_nav2_test_7/states/s_pattern_states/sti_spattern_forward_3.hpp>
#include <sm_nav2_test_7/states/s_pattern_states/sti_spattern_forward_4.hpp>
#include <sm_nav2_test_7/states/s_pattern_states/sti_spattern_loop_start.hpp>
#include <sm_nav2_test_7/states/s_pattern_states/sti_spattern_rotate_1.hpp>
#include <sm_nav2_test_7/states/s_pattern_states/sti_spattern_rotate_2.hpp>
#include <sm_nav2_test_7/states/s_pattern_states/sti_spattern_rotate_3.hpp>
#include <sm_nav2_test_7/states/s_pattern_states/sti_spattern_rotate_4.hpp>
} // namespace SS5
} // namespace sm_nav2_test_7
