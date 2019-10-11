# Install script for directory: /home/yangjian/AWS_CodeBuild/AWS_CodeBuild/Roboscrub_ws/src/scrubber_gazebo

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/yangjian/AWS_CodeBuild/AWS_CodeBuild/Roboscrub_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/yangjian/AWS_CodeBuild/AWS_CodeBuild/Roboscrub_ws/build/scrubber_gazebo/catkin_generated/installspace/scrubber_gazebo.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/scrubber_gazebo/cmake" TYPE FILE FILES
    "/home/yangjian/AWS_CodeBuild/AWS_CodeBuild/Roboscrub_ws/build/scrubber_gazebo/catkin_generated/installspace/scrubber_gazeboConfig.cmake"
    "/home/yangjian/AWS_CodeBuild/AWS_CodeBuild/Roboscrub_ws/build/scrubber_gazebo/catkin_generated/installspace/scrubber_gazeboConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/scrubber_gazebo" TYPE FILE FILES "/home/yangjian/AWS_CodeBuild/AWS_CodeBuild/Roboscrub_ws/src/scrubber_gazebo/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/scrubber_gazebo/scrubber_gazebo_cleanArea" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/scrubber_gazebo/scrubber_gazebo_cleanArea")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/scrubber_gazebo/scrubber_gazebo_cleanArea"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/scrubber_gazebo" TYPE EXECUTABLE FILES "/home/yangjian/AWS_CodeBuild/AWS_CodeBuild/Roboscrub_ws/devel/lib/scrubber_gazebo/scrubber_gazebo_cleanArea")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/scrubber_gazebo/scrubber_gazebo_cleanArea" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/scrubber_gazebo/scrubber_gazebo_cleanArea")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/scrubber_gazebo/scrubber_gazebo_cleanArea"
         OLD_RPATH "/opt/ros/melodic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/scrubber_gazebo/scrubber_gazebo_cleanArea")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/scrubber_gazebo/scrubber_gazebo_cleanArea" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/scrubber_gazebo/scrubber_gazebo_cleanArea")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/scrubber_gazebo/scrubber_gazebo_cleanArea"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/scrubber_gazebo" TYPE EXECUTABLE FILES "/home/yangjian/AWS_CodeBuild/AWS_CodeBuild/Roboscrub_ws/devel/lib/scrubber_gazebo/scrubber_gazebo_cleanArea")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/scrubber_gazebo/scrubber_gazebo_cleanArea" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/scrubber_gazebo/scrubber_gazebo_cleanArea")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/scrubber_gazebo/scrubber_gazebo_cleanArea"
         OLD_RPATH "/opt/ros/melodic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/scrubber_gazebo/scrubber_gazebo_cleanArea")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/scrubber_gazebo" TYPE DIRECTORY FILES
    "/home/yangjian/AWS_CodeBuild/AWS_CodeBuild/Roboscrub_ws/src/scrubber_gazebo/launch"
    "/home/yangjian/AWS_CodeBuild/AWS_CodeBuild/Roboscrub_ws/src/scrubber_gazebo/configuration_files"
    "/home/yangjian/AWS_CodeBuild/AWS_CodeBuild/Roboscrub_ws/src/scrubber_gazebo/maps"
    "/home/yangjian/AWS_CodeBuild/AWS_CodeBuild/Roboscrub_ws/src/scrubber_gazebo/worlds"
    )
endif()

