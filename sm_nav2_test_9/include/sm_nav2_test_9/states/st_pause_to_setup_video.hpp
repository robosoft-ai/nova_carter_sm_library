#pragma once

#include <smacc2/smacc.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <smacc2/client_base_components/cp_topic_publisher.hpp>

namespace sm_nav2_test_9 {
using namespace smacc2::default_events;
using smacc2::client_behaviors::CbSleepFor;
using namespace std::chrono_literals;
using namespace cl_nav2z;
using namespace cl_keyboard;

// STATE DECLARATION
struct StPauseToSetupVideo
    : smacc2::SmaccState<StPauseToSetupVideo , MsNav2Test1RunMode> {
  using SmaccState::SmaccState;

   // DECLARE CUSTOM OBJECT TAGS
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};


  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbSleepFor, OrNavigation>, StInitialMove, SUCCESS>,
    //Keyboard events
    Transition<EvKeyPressD<CbDefaultKeyboardBehavior, OrKeyboard>, StBatteryCheck, SUCCESS>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StInitialMove, NEXT>
    // Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, MsRecover, NEXT>
    // Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, StNavigateToWaypoint2, NEXT>
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {
    // -#configure_orthogonal<OrNavigation, CbSleepFor>(20s);
    configure_orthogonal<OrNavigation, CbSleepFor>(10s);
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure() {}

  void onEntry() {}
};
} 
