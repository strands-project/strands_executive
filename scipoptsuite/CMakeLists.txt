cmake_minimum_required(VERSION 2.8.3)

project(scipoptsuite)

include(ExternalProject)

ExternalProject_Add(
   scipoptsuite   
   PREFIX scipoptsuite
   URL http://scip.zib.de/download/release/scipoptsuite-3.0.2.tgz
   UPDATE_COMMAND ""
   PATCH_COMMAND patch -R -p0 < ${CMAKE_CURRENT_SOURCE_DIR}/scipoptsuite-3.0.2-darwin-clang.diff
   CONFIGURE_COMMAND ""
   BUILD_COMMAND make -C ${CMAKE_BINARY_DIR}/scipoptsuite/src/scipoptsuite
   INSTALL_COMMAND make -C ${CMAKE_BINARY_DIR}/scipoptsuite/src/scipoptsuite/scip-3.0.2 INSTALLDIR=${CMAKE_BINARY_DIR} installheaders installlibs
)

# need explicit unpack inorder to apply patch on OSX
ExternalProject_Add_Step(scipoptsuite extract_scip
   COMMAND ${CMAKE_COMMAND} -E tar zxvf scip-3.0.2.tgz WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/scipoptsuite/src/scipoptsuite
   DEPENDEES download
   DEPENDERS patch
)

# need explicit unpack inorder to apply patch on OSX
ExternalProject_Add_Step(scipoptsuite extract_gcg
   COMMAND ${CMAKE_COMMAND} -E tar zxvf gcg-1.1.1.tgz WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/scipoptsuite/src/scipoptsuite
   DEPENDEES download
   DEPENDERS patch
)

# need explicit unpack inorder to apply patch on OSX
ExternalProject_Add_Step(scipoptsuite extract_ug
   COMMAND ${CMAKE_COMMAND} -E tar zxvf ug-0.7.2.tgz WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/scipoptsuite/src/scipoptsuite
   DEPENDEES download
   DEPENDERS patch
)

# need explicit unpack inorder to apply patch on OSX
ExternalProject_Add_Step(scipoptsuite extract_soplex
   COMMAND ${CMAKE_COMMAND} -E tar zxvf soplex-1.7.2.tgz WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/scipoptsuite/src/scipoptsuite
   DEPENDEES download
   DEPENDERS patch
)

# need explicit unpack inorder to apply patch on OSX
ExternalProject_Add_Step(scipoptsuite extract_zimpl
   COMMAND ${CMAKE_COMMAND} -E tar zxvf zimpl-3.3.1.tgz WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/scipoptsuite/src/scipoptsuite
   DEPENDEES download
   DEPENDERS patch
)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES scipoptsuite
#  CATKIN_DEPENDS gmp readline zlib
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(scipoptsuite
#   src/${PROJECT_NAME}/scipoptsuite.cpp
# )

## Declare a cpp executable
# add_executable(scipoptsuite_node src/scipoptsuite_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(scipoptsuite_node scipoptsuite_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(scipoptsuite_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS scipoptsuite scipoptsuite_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_scipoptsuite.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)



