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

#include <multirole_sensor_client/cl_multirole_sensor.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

namespace sm_nav2_test_7
{
namespace cl_lidar
{

class ClLidarSensor : public cl_multirole_sensor::ClMultiroleSensor<sensor_msgs::LaserScan>
{
public:
    ClLidarSensor(std::string topicname, ros::Duration timeout)
    {

        this->topicName = topicname;
        this->timeout_ = timeout;
    }
};
} // namespace cl_lidar
} // namespace sm_nav2_test_7
