# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    apriltag_node = ComposableNode(
        package="isaac_ros_apriltag",
        plugin="nvidia::isaac_ros::apriltag::AprilTagNode",
        name="apriltag",
        remappings=[
            ("/image", "/front_stereo_camera/left_rgb/image_raw"),  #'/rgb_left'),
            ("/camera_info", "/front_stereo_camera/left_rgb/camerainfo"),
        ],  #'/camera_info_left')],
        # remappings=[('/image', '/rgb_left'),
        #             ('/camera_info', '/camera_info_left')],
        parameters=[{"size": 0.32, "max_tags": 64, "tile_size": 4}],
    )

    apriltag_container = ComposableNodeContainer(
        package="rclcpp_components",
        name="apriltag_container",
        namespace="",
        executable="component_container_mt",
        composable_node_descriptions=[
            apriltag_node,
        ],
        output="screen",
        # prefix=['xterm -e gdb -ex run --args']
        prefix="xterm -hold -e",
    )

    return launch.LaunchDescription([apriltag_container])
