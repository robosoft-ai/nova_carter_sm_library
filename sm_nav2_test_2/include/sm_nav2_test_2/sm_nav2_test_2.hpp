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
#include <smacc2/client_behaviors/cb_sleep_for.hpp>
#include <smacc2/client_behaviors/cb_ros_stop_2.hpp>

#include <lifecyclenode_client/client_behaviors/cb_deactivate.hpp>
#include <lifecyclenode_client/lifecyclenode_client.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>

// CLIENT BEHAVIORS
#include <ros_timer_client/client_behaviors/cb_ros_timer.hpp>
#include <ros_timer_client/client_behaviors/cb_timer_countdown_once.hpp>

#include <keyboard_client/cl_keyboard.hpp>
#include <keyboard_client/client_behaviors/cb_default_keyboard_behavior.hpp>

#include <nav2z_client/client_behaviors.hpp>
#include <nav2z_client/components/odom_tracker/cp_odom_tracker.hpp>
#include <nav2z_client/nav2z_client.hpp>
#include <nav2z_client/client_behaviors/cb_navigate_global_position.hpp>

// #include
// <sm_nav2_test_2/clients/nav2z_client/client_behaviors/cb_navigate_next_waypoint.hpp>
#include <nav2z_client/client_behaviors/cb_active_stop.hpp>
#include <nav2z_client/client_behaviors/cb_load_waypoints_file.hpp>
#include <nav2z_client/client_behaviors/cb_navigate_next_waypoint_free.hpp>
#include <nav2z_client/client_behaviors/cb_position_control_free_space.hpp>
#include <nav2z_client/client_behaviors/cb_pure_spinning.hpp>
#include <nav2z_client/client_behaviors/cb_save_slam_map.hpp>


#include <ros_publisher_client/client_behaviors/cb_default_publish_loop.hpp>
#include <ros_publisher_client/client_behaviors/cb_muted_behavior.hpp>
#include <ros_publisher_client/client_behaviors/cb_publish_once.hpp>

#include <ros_publisher_client/cl_ros_publisher.hpp>

// STATE REACTORS
#include <sr_all_events_go/sr_all_events_go.hpp>
#include <sr_conditional/sr_conditional.hpp>
#include <sr_event_countdown/sr_event_countdown.hpp>

// ORTHOGONALS
//#include <sm_nav2_test_2/orthogonals/or_assigner.hpp>
#include <sm_nav2_test_2/orthogonals/or_keyboard.hpp>
#include <sm_nav2_test_2/orthogonals/or_navigation.hpp>
#include <sm_nav2_test_2/orthogonals/or_perception.hpp>
#include <sm_nav2_test_2/orthogonals/or_lifecyclenode.hpp>
#include <sm_nav2_test_2/orthogonals/or_slam.hpp>
#include <sm_nav2_test_2/orthogonals/or_localization.hpp>
#include <sm_nav2_test_2/orthogonals/or_timer.hpp>

using namespace cl_nav2z;
using namespace smacc2::state_reactors;

namespace sm_nav2_test_2 {
// STATE FORWARD DECLARATIONS
class StAcquireSensors;
class StPauseToSetupVideo;
class StInitialMove;
class StInitialMoveStop;
class StPreSpiralPattern;
class StSpiralPattern;
class StSwitchYard;
class StRecoveryNav2;
class StSpinRight1;
class StSpinLeft1;
class StSpinLeft2;
class StWaypointSpinLeft;
class StWaypointSpinRight;
class StNavigateWarehouseWaypointsX;
class StPauseSlam;
class StFinalState;
class StFinalReturnToOrigin;
class StNavigateToWaypoint1;
class StNavigateToWaypoint2;
class StNavigateToWaypoint3;
class StNavigateToWaypoint4;
class StNavigateToWaypoint5;
class StLoadingWaypointsFile;
class StBackup1;

// SUPERSTATE FORWARD DECLARATIONS
// MODE STATES FORWARD DECLARATIONS
class MsNav2Test1RunMode;
class MsNav2Test1RecoveryMode;

namespace SS1 {
class SsRadialPattern1;
}

namespace SS2 {
class SsSPattern1;
}

namespace SS3 {
class SsFPattern1;
}

// custom sm_nav2_test_2 event
struct EvGlobalError : sc::event<EvGlobalError> {};

} // namespace sm_nav2_test_2

using namespace sm_nav2_test_2;
using namespace cl_ros_timer;
using namespace smacc2;

namespace sm_nav2_test_2 {
/// \brief Advanced example of state machine with smacc that shows multiple
/// techniques
///  for the development of state machines
struct SmNav2Test2
    : public smacc2::SmaccStateMachineBase<SmNav2Test2,
                                           MsNav2Test1RunMode> {
  typedef mpl::bool_<false> shallow_history;
  typedef mpl::bool_<false> deep_history;
  typedef mpl::bool_<false> inherited_deep_history;

  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override {
    this->createOrthogonal<OrNavigation>();
    //this->createOrthogonal<OrAssigner>();
    this->createOrthogonal<OrPerception>();
    this->createOrthogonal<OrKeyboard>();
    this->createOrthogonal<OrSlam>();
    this->createOrthogonal<OrLocalization>();
    this->createOrthogonal<OrLifecycleNode>();
    this->createOrthogonal<OrTimer>();
  }
};

} // namespace sm_nav2_test_2

// MODE STATES
#include <sm_nav2_test_2/modestates/ms_nav2_test_2_run_mode.hpp>

#include <sm_nav2_test_2/modestates/ms_nav2_test_2_recovery_mode.hpp>

// SUPERSTATES
#include <sm_nav2_test_2/superstates/ss_f_pattern_1.hpp>
#include <sm_nav2_test_2/superstates/ss_radial_pattern_1.hpp>
#include <sm_nav2_test_2/superstates/ss_s_pattern_1.hpp>

// STATES
#include <sm_nav2_test_2/states/st_acquire_sensors.hpp>
#include <sm_nav2_test_2/states/st_recovery_nav2.hpp>
#include <sm_nav2_test_2/states/st_pause_to_setup_video.hpp>
#include <sm_nav2_test_2/states/st_initial_move.hpp>
#include <sm_nav2_test_2/states/st_initial_move_stop.hpp>
#include <sm_nav2_test_2/states/st_spiral_pattern.hpp>
#include <sm_nav2_test_2/states/st_pre_spiral_pattern.hpp>
#include <sm_nav2_test_2/states/st_switch_yard.hpp>
#include <sm_nav2_test_2/states/st_final_state.hpp>
#include <sm_nav2_test_2/states/st_spin_left_1.hpp>
#include <sm_nav2_test_2/states/st_spin_left_2.hpp>
#include <sm_nav2_test_2/states/st_spin_right_1.hpp> 
#include <sm_nav2_test_2/states/st_waypoint_spin_left.hpp>
#include <sm_nav2_test_2/states/st_waypoint_spin_right.hpp>
#include <sm_nav2_test_2/states/st_navigate_warehouse_waypoints.x.hpp>
#include <sm_nav2_test_2/states/st_pause_slam.hpp>
#include <sm_nav2_test_2/states/st_navigate_to_waypoint_1.hpp>
#include <sm_nav2_test_2/states/st_navigate_to_waypoint_2.hpp>
#include <sm_nav2_test_2/states/st_navigate_to_waypoint_3.hpp>
#include <sm_nav2_test_2/states/st_navigate_to_waypoint_4.hpp>
#include <sm_nav2_test_2/states/st_navigate_to_waypoint_5.hpp>
#include <sm_nav2_test_2/states/st_loading_waypoints_file.hpp>
#include <sm_nav2_test_2/states/st_final_return_to_origin.hpp>
#include <sm_nav2_test_2/states/st_backup_1.hpp>
