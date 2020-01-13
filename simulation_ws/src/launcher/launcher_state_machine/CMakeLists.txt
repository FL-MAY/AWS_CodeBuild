cmake_minimum_required(VERSION 2.8.3)
project(launcher_state_machine)

find_package(catkin REQUIRED COMPONENTS
             actionlib
             geometry_msgs
             mbf_msgs
             roscpp
             std_srvs
             dynamic_reconfigure
             launcher_msgs
             scrubber_msgs
             costmap_2d
             db_msgs
             amcl
             tf2
             tf2_ros
             )

catkin_package(
		#  INCLUDE_DIRS include
		#  LIBRARIES launcher_state_machine
		CATKIN_DEPENDS
		geometry_msgs
		launcher_msgs
		mbf_msgs
		db_msgs
		roscpp
		scrubber_msgs
		costmap_2d
		amcl
		#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
		include
		${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(StateMachine
            src/StateMachine.cpp
            )


add_library(ScrubberStates
            src/ScrubberStates.cpp
            )


add_dependencies(ScrubberStates
		         ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

add_library(Automatic
            src/Automatic.cpp)


add_dependencies(Automatic
                 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

add_library(Mapping
            src/Mapping.cpp)


add_library(Tracking
            src/Tracking.cpp)


add_library(Manual
            src/Manual.cpp)


add_library(Report
            src/Report.cpp)


add_dependencies(Report
                 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})


target_link_libraries(Automatic ScrubberStates Report)
target_link_libraries(Mapping ScrubberStates)
target_link_libraries(Tracking ScrubberStates)
target_link_libraries(Manual  ScrubberStates Report)

target_link_libraries(StateMachine
                      ScrubberStates
                      Automatic
                      Mapping
                      Tracking
                      Manual
                      Report
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      )
add_dependencies(StateMachine
                 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context

add_executable(launcher_state_machine_node
               src/launcherStateMachineNode.cpp)

add_dependencies(launcher_state_machine_node
                 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

target_link_libraries(launcher_state_machine_node
                      StateMachine)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(
		TARGETS
		StateMachine
		launcher_state_machine_node
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

