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

#include <sm_nav2_test_7/clients/cl_foundationpose/client_behaviors/cb_track_object_pose.hpp>

namespace sm_nav2_test_7
{
  using cl_nav2z::CbNavigateGlobalPosition;
  using cl_foundationpose::CbTrackObjectPose;

// STATE DECLARATION - Calculate Final Pose
struct StRecoverStep3 : smacc2::SmaccState<StRecoverStep3, MsRecover>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct TIMEOUT : ABORT{};
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StRecoverStep4, SUCCESS>,
    Transition<EvCbSuccess<CbNavigateGlobalPosition, OrNavigation>, StRecoverStep4, SUCCESS>
    //Transition<EvCbFailure<CbNavigateGlobalPosition, OrNavigation>, StRecoverStep3, ABORT>
  
    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
  //  configure_orthogonal<OrTimer, CbTimerCountdownOnce>(50);
  //  configure_orthogonal<OrSubscriber, CbWatchdogSubscriberBehavior>();
  //  configure_orthogonal<OrUpdatablePublisher, CbDefaultPublishLoop>();
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
    configure_orthogonal<OrPerception, CbTrackObjectPose>("fp_object");
  }

  void runtimeConfigure() 
  {
    CpObjectTrackerTf* objectTracker;
    requiresComponent(objectTracker);
    
    auto pose = objectTracker->updateGlobalObjectPoseWithOffset("fp_object", "map");
    RCLCPP_INFO(getLogger(), "[StRecoverStep3] Navigating to Facing Dock pose: %f, %f, %f", pose->pose.position.x, pose->pose.position.y, tf2::getYaw(pose->pose.orientation));
    this->configure<OrNavigation, CbNavigateGlobalPosition>(pose->pose.position.x, pose->pose.position.y, tf2::getYaw(pose->pose.orientation));
  }

  void onEntry() { RCLCPP_INFO(getLogger(), "On Entry!"); }

  void onExit() { RCLCPP_INFO(getLogger(), "On Exit!"); }
};
}  // namespace sm_nav2_test_7
