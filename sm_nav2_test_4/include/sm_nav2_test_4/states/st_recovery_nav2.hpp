#pragma once

#include <smacc2/smacc.hpp>
#include <smacc2/client_behaviors/cb_wait_action_server.hpp>
#include <smacc2/client_behaviors/cb_wait_topic.hpp>
#include <smacc2/client_behaviors/cb_ros_launch_2.hpp>

namespace sm_nav2_test_4 {
using namespace std::chrono_literals;
using namespace smacc2::default_events;

using smacc2::client_behaviors::CbSleepFor;
using smacc2::client_behaviors::CbWaitActionServer;
using smacc2::client_behaviors::CbWaitTopic;
using cl_nav2z::CbWaitNav2Nodes;
using smacc2::client_behaviors::CbRosStop2;
using smacc2::client_behaviors::CbRosLaunch2;

// STATE DECLARATION
struct StRecoveryNav2
    : smacc2::SmaccState<StRecoveryNav2, MsNav2Test1RunMode> {
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT AND TRANSITION TAGS
  struct ON_STOP_LAUNCHS : SUCCESS {};
  struct SrStopNav2;

  // TRANSITION TABLE
  typedef mpl::list<
        // Transition<EvAllGo<SrAllEventsGo, SrStopNav2>, StAcquireSensors,
        //          ON_STOP_LAUNCHS>,

        Transition<EvCbSuccess<CbSleepFor, OrNavigation>, StAcquireSensors, SUCCESS>,

        Transition<EvCbSuccess<CbSleepFor, OrSlam>, StAcquireSensors, SUCCESS>
      
      >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure() 
  {
    configure_orthogonal<OrNavigation, CbRosStop2>();
    configure_orthogonal<OrSlam, CbRosStop2>();
    //configure_orthogonal<OrAssigner, CbRosStop2>();
    configure_orthogonal<OrNavigation, CbSleepFor>(5s);
    configure_orthogonal<OrSlam, CbSleepFor>(5s);

    // // Create State Reactor
    // auto srAllSensorsReady = static_createStateReactor<
    //     SrAllEventsGo,
    //     smacc2::state_reactors::EvAllGo<SrAllEventsGo, SrStopNav2>,
    //     mpl::list<
    //         EvCbSuccess<CbRosStop2, OrNavigation>,
    //         EvCbSuccess<CbRosStop2, OrSlam>
    //         >>();
  }
};
} // namespace sm_nav2_test_4
