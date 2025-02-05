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

#include <sm_nav2_test_7/clients/cl_nav2z/client_behaviors/cb_track_path_slam.hpp>
#include <nav2z_client/components/pose/cp_pose.hpp>
#include <nav2z_client/components/odom_tracker/cp_odom_tracker.hpp>

namespace sm_nav2_test_7
{

  
  CbTrackPathSLAM::CbTrackPathSLAM()
  {

  }

  void CbTrackPathSLAM::onEntry()
  {
    RCLCPP_INFO(this->getLogger(), "Pose tracker freeze reference frame");
    cl_nav2z::Pose *poseComponent;
    requiresComponent(poseComponent);
    poseComponent->unfreezeReferenceFrame();
    // poseComponent->setReferenceFrame("odom");
    
    RCLCPP_INFO(this->getLogger(), "Odom tracker clear path");
    cl_nav2z::odom_tracker::CpOdomTracker *odomTracker;
    this->requiresComponent(odomTracker);
    // odomTracker->setOdomFrame("odom");
  
    odomTracker->clearPath();
  }

  void CbTrackPathSLAM::onExit() 
  {

  }
}  // namespace sm_dancebot_mine_ue