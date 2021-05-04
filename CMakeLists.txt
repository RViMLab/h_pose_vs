cmake_minimum_required(VERSION 3.0.2)
project(h_pose_vs)

# Compile as C++17, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

# Find packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_ros_planning_interface
  actionlib
  actionlib_msgs
  eigen_conversions
  message_generation
)

# System dependencies
find_package(Eigen3 REQUIRED)

# Generate actions in the 'action' folder
add_action_files(FILES pose.action)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp moveit_ros_planning_interface actionlib actionlib_msgs eigen_conversions message_runtime
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
  DEPENDS EIGEN3
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Executables
add_executable(pose_server_node src/pose_server_node.cpp)
add_executable(pose_client_node src/pose_client_node.cpp)

# Add cmake target dependencies of the executables
add_dependencies(pose_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(pose_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(pose_server_node
  ${catkin_LIBRARIES}
)

target_link_libraries(pose_client_node
  ${catkin_LIBRARIES}
)

# Install
install(
  TARGETS pose_server_node pose_client_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
