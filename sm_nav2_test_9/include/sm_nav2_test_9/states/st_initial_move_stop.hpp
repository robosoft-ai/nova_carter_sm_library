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
struct StInitialMoveStop
    : smacc2::SmaccState<StInitialMoveStop, MsNav2Test1RunMode> {
  using SmaccState::SmaccState;

   // DECLARE CUSTOM OBJECT TAGS
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};


  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbSleepFor, OrNavigation>, StSwitchYard, SUCCESS>,
    //Keyboard events
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StSwitchYard, NEXT>,
    Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, StInitialMove, PREVIOUS>
  > reactions;

//   CpTopicPublisher<geometry_msgs::msg::Twist> *pub;

  // STATE FUNCTIONS
  static void staticConfigure() {
    configure_orthogonal<OrNavigation, CbSleepFor>(2s);
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure() {}

  void onEntry() {
    cl_nav2z::ClNav2Z* clNav;
    this->requiresClient(clNav);
    auto pub = clNav->getComponent<smacc2::components::CpTopicPublisher<geometry_msgs::msg::Twist>>();
    auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();
    twist_msg->linear.x = 0.0; 
    twist_msg->angular.z = 0.0; 
    pub->publish(*twist_msg);
  }
};
} 
