cmake_minimum_required(VERSION 3.10)
project(cango_master)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(cango_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(TARGET_NAME cango_master)

add_executable(${TARGET_NAME} 
  src/cango_master.cpp 
  src/sequence_manager.cpp 
  src/tools.cpp
)

target_include_directories(${TARGET_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${YAML_CPP_INCLUDE_DIRS}
)

ament_target_dependencies(${TARGET_NAME}
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  cango_msgs
  nav_msgs
  nav2_msgs
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(${TARGET_NAME}
  ${YAML_CPP_LIBRARIES}
)

install(TARGETS ${TARGET_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()