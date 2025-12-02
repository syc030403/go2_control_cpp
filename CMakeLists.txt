cmake_minimum_required(VERSION 3.5)
project(go2_control_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_go REQUIRED)
find_package(std_srvs REQUIRED)

# Add executable - simple_go2_walker만!
add_executable(simple_go2_walker src/simple_go2_walker.cpp)

# Add dependencies
ament_target_dependencies(simple_go2_walker
  rclcpp
  geometry_msgs
  unitree_api
  unitree_go
)

# Install executable
install(TARGETS
  simple_go2_walker
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

add_executable(manual_waypointRecorder_mk0 src/manual_waypointRecorder_mk0.cpp)
ament_target_dependencies(manual_waypointRecorder_mk0
  rclcpp 
  unitree_go
  std_srvs
)

install(TARGETS
  manual_waypointRecorder_mk0
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(manual_waypointRecorder_mk1 src/manual_waypointRecorder_mk1.cpp)
ament_target_dependencies(manual_waypointRecorder_mk1
  rclcpp 
  unitree_go
  std_srvs
)

install(TARGETS
  manual_waypointRecorder_mk1
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(waypoint_navigator_mk0 src/waypoint_navigator_mk0.cpp)

ament_target_dependencies(waypoint_navigator_mk0
  rclcpp
  geometry_msgs
  unitree_api
  unitree_go
)

install(TARGETS
  waypoint_navigator_mk0
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(waypoint_navigator_mk1 src/waypoint_navigator_mk1.cpp)

ament_target_dependencies(waypoint_navigator_mk1
  rclcpp
  geometry_msgs
  unitree_api
  unitree_go
)

install(TARGETS
  waypoint_navigator_mk1
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(waypoint_navigator_mk2 src/waypoint_navigator_mk2.cpp)

ament_target_dependencies(waypoint_navigator_mk2
  rclcpp
  geometry_msgs
  unitree_api
  unitree_go
)

install(TARGETS
  waypoint_navigator_mk2
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(waypoint_navigator_mk5 src/waypoint_navigator_mk5.cpp)

ament_target_dependencies(waypoint_navigator_mk5
  rclcpp
  geometry_msgs
  unitree_api
  unitree_go
)

install(TARGETS
  waypoint_navigator_mk5
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(waypoint_navigator_mk6 src/waypoint_navigator_mk6.cpp)

ament_target_dependencies(waypoint_navigator_mk6
  rclcpp
  geometry_msgs
  unitree_api
  unitree_go
)

install(TARGETS
  waypoint_navigator_mk6
  DESTINATION lib/${PROJECT_NAME}
)
