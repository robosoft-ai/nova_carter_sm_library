#pragma once

#include <smacc2/smacc.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <smacc2/client_base_components/cp_topic_publisher.hpp>

namespace sm_nav2_test_3 {
using namespace smacc2::default_events;
using smacc2::client_behaviors::CbSleepFor;
using namespace std::chrono_literals;
using namespace cl_nav2z;
using namespace cl_keyboard;
using namespace cl_ros_timer;


// STATE DECLARATION
struct StPreSpiralPattern
    : smacc2::SmaccState<StPreSpiralPattern, MsNav2Test1RunMode> {
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};


  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbAbortNavigation, OrNavigation>, StSpiralPattern, SUCCESS>,

    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StSpiralPattern, NEXT>
  > reactions;

//   CpTopicPublisher<geometry_msgs::msg::Twist> *pub;

  // STATE FUNCTIONS
  static void staticConfigure() {
    //configure_orthogonal<OrTimer, CbTimerCountdownOnce>(10s); I would like to use the CLtimer instead
    configure_orthogonal<OrNavigation, CbAbortNavigation>();
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure() {}

  void onEntry() {}
};
} 
