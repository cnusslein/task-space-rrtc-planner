cmake_minimum_required(VERSION 3.8)
project(task_space_rrtc_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED) # NECESSARY?
find_package(moveit_visual_tools REQUIRED) # NECESSARY?
find_package(rclcpp REQUIRED) # NECESSARY?

add_executable(task_space_rrtc_planner src/tsrrtc_example.cpp)
target_include_directories(task_space_rrtc_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(task_space_rrtc_planner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  task_space_rrtc_planner
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)

install(TARGETS task_space_rrtc_planner
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()