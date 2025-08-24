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
#include <isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp>
#include <smacc2/smacc.hpp>
#include <smacc2/smacc_client.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace cl_mission_tracker 
{
template <typename AsyncCB, typename Orthogonal>
struct EvBatteryLoad
    : sc::event<EvBatteryLoad<AsyncCB, Orthogonal>> {};

template <typename AsyncCB, typename Orthogonal>
struct EvRadialMotion
    : sc::event<EvRadialMotion<AsyncCB, Orthogonal>> {};

template <typename AsyncCB, typename Orthogonal>
struct EvSPattern
    : sc::event<EvSPattern<AsyncCB, Orthogonal>> {};

template <typename AsyncCB, typename Orthogonal>
struct EvFPattern
    : sc::event<EvFPattern<AsyncCB, Orthogonal>> {};

class ClMissionTracker : public smacc2::ISmaccClient 
{
    private:
        int decision_counter = 0;

    public:
        ClMissionTracker() {}
        void nextDecision () {decision_counter++;}
        int getDecisionCounter() {return decision_counter;}
};

} // namespace cl_apriltag_detector
