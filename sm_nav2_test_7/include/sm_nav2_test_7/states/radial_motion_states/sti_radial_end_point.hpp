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


namespace sm_nav2_test_7 {
namespace radial_motion_states {
using namespace cl_keyboard;

// STATE DECLARATION
struct StiRadialEndPoint : smacc2::SmaccState<StiRadialEndPoint, SS> {
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};

  // TRANSITION TABLE
  typedef mpl::list<

      Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StiRadialReturn,
                 SUCCESS>,
      Transition<EvCbFailure<CbNavigateForward, OrNavigation>, StiRadialEndPoint,
                 ABORT>,
     
      // Keyboard events  
      Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StiRadialReturn, NEXT>
      >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {
    // RCLCPP_INFO(getLogger(),"ssr radial end point, distance in meters: %lf",
    // SS::ray_length_meters());
    // configure_orthogonal<OrNavigation, CbPauseSlam>();
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
    configure_orthogonal<OrNavigation, CbTrackPathOdometry>();
    configure_orthogonal<OrNavigation, CbPauseSlam>();
    configure_orthogonal<OrNavigation, CbNavigateForward>(SS::ray_length_meters());
  }

  void runtimeConfigure() {

    RCLCPP_INFO(this->getLogger(),"[StiRadialEndPoint] - distance to wall: %lf", SS::ray_length_meters());

    // this->configure<OrNavigation, CbTrackPathOdometry>();
    //odom tracker
    cl_nav2z::odom_tracker::CpOdomTracker *odomTracker;
    this->requiresComponent(odomTracker);

    // RCLCPP_INFO(this->getLogger(), "Pose tracker freeze reference frame");
    // cl_nav2z::Pose *poseComponent;
    // requiresComponent(poseComponent);
    // poseComponent->freezeReferenceFrame();
    
    // // poseComponent->setReferenceFrame("odom");
    
    // RCLCPP_INFO(this->getLogger(), "Odom tracker clear path");
    // cl_nav2z::odom_tracker::CpOdomTracker *odomTracker;
    // this->requiresComponent(odomTracker);
    // // odomTracker->setOdomFrame("odom");
  
    // odomTracker->clearPath();


    // RCLCPP_INFO(this->getLogger(), "Creating motion client");
    // this->configure<OrNavigation, CbNavigateForward>(
    //     SS::ray_length_meters());

    auto *cbForwardMotion =
        this->getOrthogonal<OrNavigation>()
            ->getClientBehavior<CbNavigateForward>();

    RCLCPP_INFO(this->getLogger(), "Configuring motion client");
    auto previousGoal = odomTracker->getCurrentMotionGoal();

    if (previousGoal) {
      cbForwardMotion->options.forceInitialOrientation =
          previousGoal->pose.orientation;
      RCLCPP_ERROR_STREAM(this->getLogger(),
                          "Previous goal orientation: "
                              << previousGoal->pose.orientation.x << ", "
                              << previousGoal->pose.orientation.y << ", "
                              << previousGoal->pose.orientation.z << ", "
                              << previousGoal->pose.orientation.w);
    };

    RCLCPP_INFO(this->getLogger(), "Getting forward distance");
    ::sm_nav2_test_7::cl_lidar::ClLidarSensor * lidarClient;
      this->requiresClient(lidarClient);

    if(lidarClient)
    {
      auto lidarData = lidarClient->getComponent<::sm_nav2_test_7::cl_lidar::CpForwardObstacleDetector>();

      auto forwardDistance = lidarData->getForwardDistance()-1.0;
      //forwardDistance = 4.0;
      cbForwardMotion->setForwardDistance( forwardDistance);
      RCLCPP_INFO(
        this->getLogger(), "Going forward in Radial pattern, (CpForwardObstacleDetector) distance to wall: %lf",
        //lidarData->getForwardDistance());
        forwardDistance);
    }
    else
    {
      RCLCPP_WARN(this->getLogger(), "No lidar and cp forward obstacle detector available");
      cbForwardMotion->setForwardDistance(1.0);
      RCLCPP_WARN(
        this->getLogger(), "Going forward in Radial pattern, distance to wall: %lf",
        SS::ray_length_meters());
    }

    // RCLCPP_INFO(this->getLogger(), "Configuring Slam Pause");
    // this->configure<OrNavigation, CbPauseSlam>();

  }
};
} // namespace radial_motion_states
} // namespace sm_nav2_test_7
