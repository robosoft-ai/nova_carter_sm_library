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
#include <smacc2/smacc_orthogonal.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <nav2z_client/components/goal_checker_switcher/cp_goal_checker_switcher.hpp>
#include <nav2z_client/components/odom_tracker/cp_odom_tracker.hpp>
#include <nav2z_client/components/pose/cp_pose.hpp>
#include <nav2z_client/components/slam_toolbox/cp_slam_toolbox.hpp>
#include <nav2z_client/components/waypoints_navigator/cp_waypoints_navigator.hpp>

#include <smacc2/client_bases/smacc_ros_launch_client_2.hpp>

namespace sm_nav2_test_6 {
using namespace cl_nav2z;
using namespace smacc2;
using namespace client_bases;
using namespace std::chrono_literals;

class OrSlam : public smacc2::Orthogonal<OrSlam> {
public:
  void onInitialize() override {

    auto rosLaunchClient = this->createClient<ClRosLaunch2>();

  }
};
} // namespace sm_nav2_test_6
