# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "amcl: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(amcl_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/longyue/git_repo/RoboScrub_Nav/src/navigation/amcl/srv/RectPara.srv" NAME_WE)
add_custom_target(_amcl_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "amcl" "/home/longyue/git_repo/RoboScrub_Nav/src/navigation/amcl/srv/RectPara.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(amcl
  "/home/longyue/git_repo/RoboScrub_Nav/src/navigation/amcl/srv/RectPara.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/amcl
)

### Generating Module File
_generate_module_cpp(amcl
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/amcl
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(amcl_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(amcl_generate_messages amcl_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/longyue/git_repo/RoboScrub_Nav/src/navigation/amcl/srv/RectPara.srv" NAME_WE)
add_dependencies(amcl_generate_messages_cpp _amcl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(amcl_gencpp)
add_dependencies(amcl_gencpp amcl_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS amcl_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(amcl
  "/home/longyue/git_repo/RoboScrub_Nav/src/navigation/amcl/srv/RectPara.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/amcl
)

### Generating Module File
_generate_module_eus(amcl
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/amcl
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(amcl_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(amcl_generate_messages amcl_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/longyue/git_repo/RoboScrub_Nav/src/navigation/amcl/srv/RectPara.srv" NAME_WE)
add_dependencies(amcl_generate_messages_eus _amcl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(amcl_geneus)
add_dependencies(amcl_geneus amcl_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS amcl_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(amcl
  "/home/longyue/git_repo/RoboScrub_Nav/src/navigation/amcl/srv/RectPara.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/amcl
)

### Generating Module File
_generate_module_lisp(amcl
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/amcl
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(amcl_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(amcl_generate_messages amcl_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/longyue/git_repo/RoboScrub_Nav/src/navigation/amcl/srv/RectPara.srv" NAME_WE)
add_dependencies(amcl_generate_messages_lisp _amcl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(amcl_genlisp)
add_dependencies(amcl_genlisp amcl_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS amcl_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(amcl
  "/home/longyue/git_repo/RoboScrub_Nav/src/navigation/amcl/srv/RectPara.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/amcl
)

### Generating Module File
_generate_module_nodejs(amcl
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/amcl
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(amcl_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(amcl_generate_messages amcl_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/longyue/git_repo/RoboScrub_Nav/src/navigation/amcl/srv/RectPara.srv" NAME_WE)
add_dependencies(amcl_generate_messages_nodejs _amcl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(amcl_gennodejs)
add_dependencies(amcl_gennodejs amcl_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS amcl_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(amcl
  "/home/longyue/git_repo/RoboScrub_Nav/src/navigation/amcl/srv/RectPara.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/amcl
)

### Generating Module File
_generate_module_py(amcl
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/amcl
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(amcl_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(amcl_generate_messages amcl_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/longyue/git_repo/RoboScrub_Nav/src/navigation/amcl/srv/RectPara.srv" NAME_WE)
add_dependencies(amcl_generate_messages_py _amcl_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(amcl_genpy)
add_dependencies(amcl_genpy amcl_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS amcl_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/amcl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/amcl
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(amcl_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/amcl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/amcl
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(amcl_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/amcl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/amcl
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(amcl_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/amcl)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/amcl
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(amcl_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/amcl)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/amcl\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/amcl
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(amcl_generate_messages_py std_msgs_generate_messages_py)
endif()
