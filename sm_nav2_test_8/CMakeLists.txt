cmake_minimum_required(VERSION 3.8)
project(sm_nav2_test_8)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

find_package(ament_cmake_auto REQUIRED)
find_package(vision_msgs REQUIRED)

ament_auto_find_build_dependencies()

include_directories(include ${vision_msgs_INCLUDE_DIRS})

ament_export_include_directories(include)

ament_auto_add_executable(${PROJECT_NAME}_node
                          src/sm_nav2_test_8/sm_nav2_test_8.cpp
                          src/clients/nav2z_client/client_behaviors/cb_position_control_free_space.cpp
                          src/clients/nav2z_client/client_behaviors/cb_track_path_odometry.cpp
                          src/clients/nav2z_client/client_behaviors/cb_track_path_slam.cpp
                          src/clients/nav2z_client/client_behaviors/cb_spiral_motion.cpp
                          # src/sm_nav2_test_8/clients/cl_foundationpose.hpp
                          # src/sm_nav2_test_8/components/com_foundationpose.cpp

)



target_link_libraries(${PROJECT_NAME}_node
  ${Boost_LIBRARIES}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  config
  maps
  DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

# install(DIRECTORY
#   scripts
#   DESTINATION lib/${PROJECT_NAME}
# )

install(PROGRAMS
  scripts/lidar_completion.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_node

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


ament_package()
