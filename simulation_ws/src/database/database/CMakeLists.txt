cmake_minimum_required(VERSION 2.8.3)
project(database)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             std_srvs
             nav_msgs
             tf2
             db_msgs
             )

catkin_package(
		#INCLUDE_DIRS include
		#  LIBRARIES launcher_state_machine
		CATKIN_DEPENDS roscpp db_msgs nav_msgs tf2
		#DEPENDS lsqlite3
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


add_library(DatabaseHelper
            src/DatabaseHelper.cpp)

target_link_libraries(DatabaseHelper
                      sqlite3
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES})

add_dependencies(DatabaseHelper
                 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

add_executable(scrubber_database
               src/scrubberDatabase.cpp)

add_dependencies(scrubber_database
                 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

target_link_libraries(scrubber_database DatabaseHelper)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context



#############
## Install ##
#############

## Mark executables and/or libraries for installation
#install(
		#TARGETS
		#ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		#LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		#RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
install(DIRECTORY database/include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

