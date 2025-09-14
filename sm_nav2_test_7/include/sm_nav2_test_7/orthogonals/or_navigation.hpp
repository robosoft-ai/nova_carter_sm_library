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

#pragma once

#include <nav2z_client/nav2z_client.hpp>
#include <smacc2/client_bases/smacc_publisher_client.hpp>
#include <smacc2/smacc_orthogonal.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <nav2z_client/components/goal_checker_switcher/cp_goal_checker_switcher.hpp>
#include <nav2z_client/components/odom_tracker/cp_odom_tracker.hpp>
#include <nav2z_client/components/pose/cp_pose.hpp>
#include <nav2z_client/components/slam_toolbox/cp_slam_toolbox.hpp>
#include <nav2z_client/components/waypoints_navigator/cp_waypoints_navigator.hpp>

#include <smacc2/client_bases/smacc_ros_launch_client_2.hpp>

#include <smacc2/client_core_components/cp_topic_publisher.hpp>

#include <sm_nav2_test_7/clients/cl_lidar/cl_lidar.hpp>
#include <sm_nav2_test_7/clients/cl_lidar/components/cp_forward_obstacle_detector.hpp>

namespace sm_nav2_test_7 {
using namespace cl_nav2z;
using namespace smacc2;
using namespace client_bases;
using namespace std::chrono_literals;

class OrNavigation : public smacc2::Orthogonal<OrNavigation> {
public:
  void onInitialize() override {
    auto nav2zClient = this->createClient<ClNav2Z>();

    auto rosLaunchClient = this->createClient<ClRosLaunch2>();

    // auto publisherClient = this->createClient<SmaccPublisherClient>();

    // create pose component
    nav2zClient->createComponent<cl_nav2z::Pose>(cl_nav2z::StandardReferenceFrames::Map);

    // create planner switcher
    nav2zClient->createComponent<CpPlannerSwitcher>();

    // create goal checker switcher
    nav2zClient->createComponent<cl_nav2z::CpGoalCheckerSwitcher>();

    // create odom tracker
    nav2zClient->createComponent<cl_nav2z::odom_tracker::CpOdomTracker>("notopic","map", cl_nav2z::odom_tracker::OdomTrackerStrategy::POSE_COMPONENT);

    // create odom tracker
    nav2zClient->createComponent<cl_nav2z::CpSlamToolbox>();

    nav2zClient->createComponent<smacc2::client_core_components::CpTopicPublisher<geometry_msgs::msg::Twist>>("/cmd_vel");
    
    // create waypoints navigator component
    auto waypointsNavigator =
        nav2zClient->createComponent<CpWaypointNavigator>();

    // nav2zClient->createComponent<sm_nav2_test_7::CpUEPose>("/ue_ros/map_origin_entity_state");

    /*auto waypointsNavigator = */
    // nav2zClient->createComponent<::cl_nav2z::CpWaypointNavigatorBase>();
  }
};
} // namespace sm_nav2_test_7
