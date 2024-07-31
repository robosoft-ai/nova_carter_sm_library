#!/bin/bash

source /opt/ros/humble/setup.bash

ros2 topic pub /clicked_point geometry_msgs/msg/PointStamped "
header:
  stamp:
    sec: 1696417411
    nanosec: 172037944
  frame_id: map
point:
  x: -30.2908992767334
  y: 44.03512191772461
  z: 0.006378173828125
" --once

ros2 topic pub /clicked_point geometry_msgs/msg/PointStamped "
header:
  stamp:
    sec: 1696417892
    nanosec: 837120351
  frame_id: map
point:
  x: 29.334697723388672
  y: 43.96137237548828
  z: 0.002471923828125
"

ros2 topic pub /clicked_point geometry_msgs/msg/PointStamped "
header:
  stamp:
    sec: 1696418056
    nanosec: 487895629
  frame_id: map
point:
  x: 28.936553955078125
  y: -30.039058685302734
  z: -0.001434326171875
" --once

ros2 topic pub /clicked_point geometry_msgs/msg/PointStamped "
header:
  stamp:
    sec: 1696418385
    nanosec: 67372988
  frame_id: map
point:
  x: -29.617624282836914
  y: -29.65144920349121
  z: -0.001434326171875
" --once

ros2 topic pub /clicked_point geometry_msgs/msg/PointStamped "
header:
  stamp:
    sec: 1696418581
    nanosec: 701335763
  frame_id: map
point:
  x: -0.2395467907190323
  y: 0.038018785417079926
  z: 0.002471923828125
" --once
