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

#include <smacc2/smacc.hpp>
#include <sm_nav2_test_6/clients/cl_foundationpose/cl_foundationpose.hpp>
#include <sm_nav2_test_6/clients/cl_foundationpose/components/cp_object_tracker_1.hpp>


namespace sm_nav2_test_6 {

using namespace cl_foundationpose;

class OrPerception : public smacc2::Orthogonal<OrPerception> {
public:
  void onInitialize() override 
  {
      auto client = this->createClient<ClFoundationPose>();

      //configure the client
       auto subcomponent = client->createComponent<CpTopicSubscriber<vision_msgs::msg::Detection3DArray>>("/detection3d_array");    

      client->createComponent<CpObjectTracker1>();

  }
};
} // namespace sm_nav2_test_6
