# Copyright 2024 Robosoft Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This is all-in-one launch script intended for use by nav2 developers."""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    xtermprefix = "xterm -xrm 'XTerm*scrollBar:  true' -xrm 'xterm*rightScrollBar: true' -hold -geometry 1000x600 -sl 10000 -e"

    # Get the launch directory
    sm_nav2_test_6_dir = get_package_share_directory("sm_nav2_test_6")
    sm_nav2_test_6_launch_dir = os.path.join(sm_nav2_test_6_dir, "launch")

    # nav_dir = get_package_share_directory("carter_navigation")
    # nav_dir_launch = os.path.join(nav_dir, "launch")

    # Create the launch configuration variables
    slam = LaunchConfiguration("slam")
    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    active_slam_sel = LaunchConfiguration("active_slam_sel")

    params_file = LaunchConfiguration("params_file")
    default_nav_to_pose_bt_xml = LaunchConfiguration("default_nav_to_pose_bt_xml")
    autostart = LaunchConfiguration("autostart")
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace",
        default_value="false",
        description="Whether to apply a namespace to the navigation stack",
    )

    declare_show_gz_lidar = DeclareLaunchArgument(
        "show_gz_lidar",
        default_value="true",
        description="Whether to apply a namespace to the navigation stack",
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam", default_value="True", description="Whether run a SLAM"
    )

    declare_active_slam_sel = DeclareLaunchArgument(  # Added to disable SLAM or alternatives
        "active_slam_sel",
        default_value="False",
        description="Active selection of slam or alternative mode",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation clock if true"
    )

    declare_gazebo_headless_cmd = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Use headless Gazebo if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(sm_nav2_test_6_dir, "config", "nav2_config.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_bt_xml_cmd = DeclareLaunchArgument(
        "default_nav_to_pose_bt_xml",
        default_value=os.path.join(sm_nav2_test_6_dir, "config", "default_nav_to_pose_bt.xml"),
        description="Full path to the behavior tree xml file to use",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart", default_value="true", description="Automatically startup the nav2 stack"
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(sm_nav2_test_6_dir, "rviz", "nav2_default_view.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        "use_robot_state_pub",
        default_value="True",
        description="Whether to start the robot state publisher",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(
            sm_nav2_test_6_dir, "maps", "improved_carter_warehouse_navigation.yaml"
        ),
        # default_value=os.path.join(nav_dir, "maps", "carter_warehouse_navigation.yaml"),
        description="Full path to map file to load",
    )


    sm_nav2_test_6_node = Node(
        package="sm_nav2_test_6",
        executable="sm_nav2_test_6_node",
        name="SmNav2Test6",
        output="screen",
        # prefix=xtermprefix + " gdb -ex run --args",
        prefix="xterm -hold -e",
        parameters=[
            os.path.join(
                get_package_share_directory("sm_nav2_test_6"),
                "config/sm_nav2_test_6_config.yaml",
            )
        ],
        remappings=[
            # ("/odom", "/odometry/filtered"),
            # ("/sm_nav2_test_6_2/odom_tracker/odom_tracker_path", "/odom_tracker_path"),
            # ("/sm_nav2_test_6_2/odom_tracker/odom_tracker_stacked_path", "/odom_tracker_path_stacked")
        ],
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    keyboard_client_node = Node(
        package="keyboard_client",
        executable="keyboard_server_node.py",
        name="keyboard_client",
        output="screen",
        prefix="xterm -hold -e",
        arguments=["--ros-args", "--log-level", "INFO"],
    )


    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_gazebo_headless_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_show_gz_lidar)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)

    ld.add_action(sm_nav2_test_6_node)
    ld.add_action(declare_active_slam_sel)
    ld.add_action(keyboard_client_node)

    return ld
