cmake_minimum_required(VERSION 3.1.2)
project(gazebo_rad_obstacle)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  gazebo_ros
  gazebo_plugins
  gazebo_rad_msgs
  )

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

find_package(gazebo REQUIRED)

catkin_package(
  DEPENDS gazebo_ros gazebo_rad_msgs
  CATKIN_DEPENDS roscpp std_msgs
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  )

add_library(
  gazebo_rad_obstacle SHARED
  src/gazebo_rad_obstacle.cpp
  )

target_link_libraries(
  gazebo_rad_obstacle
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  )

add_dependencies(
  gazebo_rad_obstacle
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

install(DIRECTORY gazebo_files
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

# add_custom_command(TARGET gazebo_rad_obstacle
#   WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
#   COMMAND bash -c "${CMAKE_CURRENT_SOURCE_DIR}/gazebo_models/build.sh"
#   )
