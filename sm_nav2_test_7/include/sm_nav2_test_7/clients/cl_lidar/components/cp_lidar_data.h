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
#include <smacc2/component.hpp>
#include <smacc2/smacc_updatable.hpp>

namespace sm_nav2_test_7
{
namespace cl_lidar
{
class CpLidarSensorData : public smacc2::ISmaccComponent, public smacc2::ISmaccUpdatable
{
public:
  sensor_msgs::LaserScan lastMessage_;
  float forwardObstacleDistance;

  const float SECURITY_DISTANCE =  1; //meters

  virtual void onInitialize() override
  {
    auto client_ = dynamic_cast<smacc2::client_bases::SmaccSubscriberClient<sensor_msgs::LaserScan> *>(owner_);
    client_->onMessageReceived(&CpLidarSensorData::MessageCallbackStoreDistanceToWall, this);
  }

  void MessageCallbackStoreDistanceToWall(const sensor_msgs::LaserScan &scanmsg)
  {
    this->lastMessage_ = scanmsg;
    auto fwdist = scanmsg.ranges[scanmsg.ranges.size() / 2] /*meter*/;

    /*if the distance is infinity or nan, use max range distance*/
    if (fwdist == std::numeric_limits<float>::infinity() || fwdist != fwdist)
    {
      this->forwardObstacleDistance = scanmsg.range_max - SECURITY_DISTANCE /*meters*/;
      RCLCPP_INFO_THROTTLE(getLogger(), *getNode(), 5000, "[CpLidarSensorData] Distance to forward obstacle is not a number, setting default value to: %lf", scanmsg.range_max);
      //ROS_INFO_THROTTLE(1.0, "[CpLidarSensorData] Distance to forward obstacle is not a number, setting default value to: %lf", scanmsg.range_max);
    }
    else
    {
      this->forwardObstacleDistance = std::max(fwdist - SECURITY_DISTANCE, 0.0F);
    }
  }
};
} // namespace cl_lidar
} // namespace sm_nav2_test_7
