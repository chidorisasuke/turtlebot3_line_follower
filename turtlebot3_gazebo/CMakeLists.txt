################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_gazebo)
cmake_policy(SET CMP0054 NEW)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  gazebo_ros
  cv_bridge
#  message_generation

)

find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)


################################################################################
# Setup for python modules and scripts
################################################################################
catkin_python_setup()


################################################################################
# Declare ROS messages, services and actions
################################################################################
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)
################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs tf gazebo_ros rospy
  DEPENDS GAZEBO
)

################################################################################
# Build
################################################################################
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_executable(turtlebot3_drive src/turtlebot3_drive.cpp)
add_dependencies(turtlebot3_drive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(turtlebot3_drive ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS turtlebot3_drive
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch models rviz worlds 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  src/robot.py
  src/simple_line_follow.py
#  src/path_ekf_plotter.py
  src/path_odom_plotter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

################################################################################
# Test
################################################################################
