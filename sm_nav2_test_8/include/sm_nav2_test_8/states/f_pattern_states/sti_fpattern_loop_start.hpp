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

namespace sm_nav2_test_8 {
namespace f_pattern_states {
// STATE DECLARATION
template <typename SS>
struct StiFPatternStartLoop : smacc2::SmaccState<StiFPatternStartLoop<SS>, SS> {
  typedef SmaccState<StiFPatternStartLoop<SS>, SS> TSti;
  using TSti::context_type;
  using TSti::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

      Transition<EvLoopContinue<StiFPatternStartLoop<SS>>,
                 StiFPatternRotate1<SS>, CONTINUELOOP>

      >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {}

  bool loopCondition() {
    auto &superstate = TSti::template context<SS>();
    return superstate.iteration_count++ < superstate.total_iterations();
  }

  void onEntry() {
    TSti::checkWhileLoopConditionAndThrowEvent(
        &StiFPatternStartLoop<SS>::loopCondition);
  }
};
} // namespace f_pattern_states
} // namespace sm_nav2_test_8
