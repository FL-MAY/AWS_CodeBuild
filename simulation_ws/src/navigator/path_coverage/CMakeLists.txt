cmake_minimum_required(VERSION 2.8.3)
project(path_coverage)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  angles
  costmap_2d
  roscpp
  rospy
  std_msgs
  std_srvs
  nav_msgs
  tf
  geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${catkin_INCLUDE_DIRS}
                    include
                    ${Eigen3_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS})

add_service_files(
  FILES
  GetPathInZone.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime costmap_2d geometry_msgs nav_msgs roscpp tf message_generation std_msgs std_srvs rospy
  LIBRARIES path_coverage
  DEPENDS
)

add_executable(path_planning_node src/PathPlanningNode.cpp src/CleaningPathPlanner.cpp src/trajectory_generator_waypoint.cpp)
add_dependencies(path_planning_node ${PROJECT_NAME}_generate_messages_cpp)
#add_executable(get_path_test src/GetPathTest.cpp)
target_link_libraries(path_planning_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
#target_link_libraries(get_path_test ${catkin_LIBRARIES} ${OpenCV_LIBS})
