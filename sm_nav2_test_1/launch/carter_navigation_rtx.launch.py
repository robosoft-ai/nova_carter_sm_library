## Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
## NVIDIA CORPORATION and its licensors retain all intellectual property
## and proprietary rights in and to this software, related documentation
## and any modifications thereto.  Any use, reproduction, disclosure or
## distribution of this software and related documentation without an express
## license agreement from NVIDIA CORPORATION is strictly prohibited.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    xtermprefix = "xterm -xrm 'XTerm*scrollBar:  true' -xrm 'xterm*rightScrollBar: true' -hold -geometry 1000x600 -sl 10000 -e"

    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    active_slam_sel = LaunchConfiguration("active_slam_sel", default="False")

    slam = LaunchConfiguration("slam", default="True")

    map_dir = LaunchConfiguration(
        "map",
        default=os.path.join(
            get_package_share_directory("sm_nav2_test_1"),
            "maps",
            "improved_carter_warehouse_navigation.yaml"
            # get_package_share_directory("carter_navigation"), "maps", "carter_warehouse_navigation.yaml"
        ),
    )

    param_dir = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            get_package_share_directory("sm_nav2_test_1"),
            "config",
            "rtx_carter_navigation_params.yaml",
        ),
    )

    # nav2_bringup_launch_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")
    isaac_exploration_launch_dir = os.path.join(
        get_package_share_directory("sm_nav2_test_1"), "launch"
    )

    # rviz_config_dir = os.path.join(get_package_share_directory("carter_navigation"), "rviz2", "carter_navigation.rviz")
    rviz_config_dir = os.path.join(
        get_package_share_directory("sm_nav2_test_1"), "config", "rtx_rviz_conf.rviz"
    )

    default_nav_to_pose_bt_xml = LaunchConfiguration("default_nav_to_pose_bt_xml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(  # Añadido para desactivar slam o alternativa
                "active_slam_sel",
                default_value="False",
                description="Active selection of slam or alternative mode",
            ),
            DeclareLaunchArgument(
                "map", default_value=map_dir, description="Full path to map file to load"
            ),
            # DeclareLaunchArgument(
            #     "map", default_value=map_dir, description="Full path to map file to load"
            # ),
            DeclareLaunchArgument(
                "params_file",
                default_value=param_dir,
                description="Full path to param file to load",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="True",
                description="Use simulation (Omniverse Isaac Sim) clock if true",
            ),
            DeclareLaunchArgument(
                "default_nav_to_pose_bt_xml",
                default_value=os.path.join(
                    get_package_share_directory("sm_nav2_test_1"),
                    "config",
                    "navigation_tree.xml",
                ),
                description="Full path to the behavior tree xml file to use",
            ),
            DeclareLaunchArgument("slam", default_value="True", description="Whether run a SLAM"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(isaac_exploration_launch_dir, "rviz_launch.py")
                ),
                launch_arguments={
                    "namespace": "",
                    "use_namespace": "False",
                    "rviz_config": rviz_config_dir,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [isaac_exploration_launch_dir, "/bringup_launch.py"]
                ),
                launch_arguments={
                    "map": map_dir,
                    "slam": slam,
                    "active_slam_sel": active_slam_sel,
                    "use_sim_time": use_sim_time,
                    "params_file": param_dir,
                }.items(),
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                # remappings=[("cloud_in", ["/front_3d_lidar/point_cloud"]), ("scan", ["/scan"])],
                remappings=[("cloud_in", ["/front_3d_lidar/point_cloud"])],
                parameters=[
                    {
                        "target_frame": "front_3d_lidar",
                        "transform_tolerance": 0.01,
                        "min_height": -0.4,  # this height from lidar recognizes pallets on the floor
                        "max_height": 2.0,
                        "angle_min": -3.1416,  # -1.5708,  # -M_PI/2 # -3.1416, #
                        "angle_max": 3.1416,  # 1.5708,  # M_PI/2 # 3.1416, #
                        "angle_increment": 0.0087,  # M_PI/360.0
                        "scan_time": 0.3333,
                        "range_min": 0.45,
                        "range_max": 100.0,
                        "use_inf": True,
                        "inf_epsilon": 1.0,
                    }
                ],
                name="pointcloud_to_laserscan",
            ),
            Node(
                package="nav2z_client",
                executable="lidar_completion.py",
                name="lidar_completion",
                output="screen",
                # remappings=[("/scan_input", "/scan2"), ("/scan_output", "/scan")],
                prefix=xtermprefix

            ),
        ]
    )
