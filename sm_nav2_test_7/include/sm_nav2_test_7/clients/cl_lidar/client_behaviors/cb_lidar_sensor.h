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

#include <multirole_sensor_client/client_behaviors/cb_default_multirole_sensor_behavior.h>
#include <sensor_msgs/LaserScan.h>
#include <sm_nav2_test_7/clients/cl_lidar/cl_lidar.h>

namespace sm_dance_bot_strikes_back
{
  namespace cl_lidar
  {
    struct CbLidarSensor : cl_multirole_sensor::CbDefaultMultiRoleSensorBehavior<sm_nav2_test_7::cl_lidar::ClLidarSensor>
    {
    public:
      CbLidarSensor()
      {
        RCLCPP_INFO(getLogger(), "CbLidarSensor Constructor");
        //ROS_INFO("CbLidarSensor Constructor");
      }

      virtual void onEntry() override
      {
        RCLCPP_INFO(getLogger(), "CbLidarSensor onEntry");
        //ROS_INFO("CbLidarSensor onEntry");
        cl_multirole_sensor::CbDefaultMultiRoleSensorBehavior<ClLidarSensor>::onEntry();
      }

      virtual void onMessageCallback(const sensor_msgs::LaserScan &msg) override
      {
      }
    };
  } // namespace cl_lidar
} // namespace sm_nav2_test_7
