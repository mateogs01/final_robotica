cmake_minimum_required(VERSION 3.5)
project(lazo_cerrado)

# Enable C++11 support
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   tf2_ros
#   lazo_abierto
#   robmovil_msgs
#   cmake_modules
#   angles
# )
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robmovil_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robmovil_ekf
#   CATKIN_DEPENDS roscpp tf2_ros lazo_abierto
#  DEPENDS system_lib
#)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
)

## Declare a C++ executable
###########
## Build ##
###########

# ===== LIBRERÍA =====
add_library(trajectory_controller
  src/KinematicPositionController.cpp
  src/TrajectoryFollower.cpp
)

ament_target_dependencies(
  trajectory_controller
  rclcpp
  geometry_msgs
  robmovil_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
)

target_include_directories(trajectory_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(lazo_cerrado::trajectory_controller ALIAS trajectory_controller)


# ===== EXECUTABLE: trajectory_follower_cl =====
add_executable(trajectory_follower_cl
  src/kinematic_position_controller_node.cpp
)

ament_target_dependencies(
  trajectory_follower_cl
  rclcpp
)

target_link_libraries(
  trajectory_follower_cl
  trajectory_controller
)


# ===== EXECUTABLE: trajectory_generator =====
add_executable(trajectory_generator
  src/trajectory_generator_node.cpp
)

ament_target_dependencies(
  trajectory_generator
  rclcpp
  geometry_msgs
  robmovil_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)


# ===== EXECUTABLE: logger =====
add_executable(logger
  src/logger_node.cpp
)

ament_target_dependencies(
  logger
  rclcpp
)

target_link_libraries(
  logger
  trajectory_controller
)

target_include_directories(trajectory_follower_cl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install targets
install(TARGETS
  trajectory_controller
  EXPORT export_trajectory_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  trajectory_follower_cl
  trajectory_generator
  logger
  DESTINATION lib/${PROJECT_NAME}
)

# Install include files
#install(DIRECTORY include/
#  DESTINATION include
#)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_targets(export_trajectory_controller HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp geometry_msgs robmovil_msgs nav_msgs tf2 tf2_geometry_msgs)


ament_package()
