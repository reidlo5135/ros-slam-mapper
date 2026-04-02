cmake_minimum_required(VERSION 3.8)
project(slam_scan_matcher VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME}
  src/slam_scan_matcher/scan_matcher.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
)

add_executable(slam_scan_matcher_node
  src/slam_scan_matcher/slam_scan_matcher.cpp
  src/main.cpp
)

set_target_properties(slam_scan_matcher_node PROPERTIES OUTPUT_NAME slam_scan_matcher)

target_include_directories(slam_scan_matcher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(slam_scan_matcher_node
  ${PROJECT_NAME}
)

ament_target_dependencies(slam_scan_matcher_node
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  ${PROJECT_NAME}
  slam_scan_matcher_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
