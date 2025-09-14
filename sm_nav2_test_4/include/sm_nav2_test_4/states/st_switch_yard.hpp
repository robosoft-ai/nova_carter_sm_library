#pragma once

#include <smacc2/smacc.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <smacc2/client_core_components/cp_topic_publisher.hpp>

namespace sm_nav2_test_4 {
using namespace smacc2::default_events;
using smacc2::client_behaviors::CbSleepFor;
using namespace std::chrono_literals;
using namespace cl_nav2z;
using namespace cl_keyboard;

// STATE DECLARATION
struct StSwitchYard
    : smacc2::SmaccState<StSwitchYard, MsNav2Test1RunMode> {
  using SmaccState::SmaccState;

   // DECLARE CUSTOM OBJECT TAGS
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};


  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbSleepFor, OrNavigation>, StSpinRight1, SUCCESS>,
    //Keyboard events
    Transition<EvKeyPressR<CbDefaultKeyboardBehavior, OrKeyboard>, StNavigateToWaypoint2, NEXT>,
    Transition<EvKeyPressS<CbDefaultKeyboardBehavior, OrKeyboard>, StNavigateToWaypoint3, NEXT>,
    Transition<EvKeyPressF<CbDefaultKeyboardBehavior, OrKeyboard>, StNavigateToWaypoint4, NEXT>,
    Transition<EvKeyPressB<CbDefaultKeyboardBehavior, OrKeyboard>, StBackup1, NEXT>,
    Transition<EvKeyPressD<CbDefaultKeyboardBehavior, OrKeyboard>, MsRecover, NEXT>,

    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StSpinRight1, NEXT>,
    Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, StInitialMoveStop, PREVIOUS>
  > reactions;

//   CpTopicPublisher<geometry_msgs::msg::Twist> *pub;

  // STATE FUNCTIONS
  static void staticConfigure() {
    configure_orthogonal<OrNavigation, CbSleepFor>(15s);
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure() {}

  void onEntry() {}
};
} 
