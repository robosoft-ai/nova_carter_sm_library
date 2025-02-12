#pragma once

#include <smacc2/smacc.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <smacc2/client_base_components/cp_topic_publisher.hpp>

namespace sm_nav2_test_7 {
using namespace smacc2::default_events;
using smacc2::client_behaviors::CbSleepFor;
using namespace std::chrono_literals;
using namespace cl_nav2z;
using namespace cl_keyboard;
using namespace cl_ros_timer;


// STATE DECLARATION
struct StSpiralPattern1
    : smacc2::SmaccState<StSpiralPattern1, MsNav2Test1RunMode> {
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};


  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvCbSuccess<CbSleepFor, OrNavigation>, StFinalReturnToOrigin, SUCCESS>,
    Transition<EvCbSuccess<CbTimerCountdownOnce, OrTimer>, StFinalReturnToOrigin, SUCCESS>,

    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StFinalReturnToOrigin, NEXT>
  > reactions;

//   CpTopicPublisher<geometry_msgs::msg::Twist> *pub;

  // STATE FUNCTIONS
  static void staticConfigure() {
    //configure_orthogonal<OrTimer, CbTimerCountdownOnce>(10s); I would like to use the CLtimer instead
    configure_orthogonal<OrNavigation, CbSleepFor>(19.25s);
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
    configure_orthogonal<OrNavigation, CbTrackPathOdometry>();
    configure_orthogonal<OrNavigation, CbPauseSlam>();

    CbSpiralMotionOptions options;
    options.linearVelocity = 0.0f;
    options.maxLinearVelocity = 0.5f;
    options.initialAngularVelocity = 1.5f;
    options.spiralMotionDuration = rclcpp::Duration::from_seconds(40);
    options.finalRadius = 20.0f;
    configure_orthogonal<OrNavigation, CbSpiralMotion>(options);

  }

  void runtimeConfigure() {}

  void onEntry() {}
};
} 
