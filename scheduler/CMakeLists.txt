cmake_minimum_required(VERSION 2.8.3)
project(scheduler)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    mongodb_store
    roscpp 
	  scipoptsuite 
	  strands_executive_msgs
)

# MESSAGE("scipoptsuite_INCLUDE_DIRS: " ${scipoptsuite_INCLUDE_DIRS})

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
#  LIBRARIES scheduler
#  CATKIN_DEPENDS scipoptsuite
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
# add_library(scheduler
#   src/${PROJECT_NAME}/scheduler.cpp
# )

link_directories(${catkin_LINK_DIRS})

add_definitions(-std=c++0x -DNDEBUG -DWITH_SCIPDEF -DROUNDING_FE -DNPARASCIP -DWITH_ZLIB -DWITH_GMP -DWITH_READLINE -DWITH_ZIMPL -fomit-frame-pointer  -mtune=native -m64)

add_library(scheduler src/task.cpp src/scheduler.cpp src/scipUser.cpp src/pairs.cpp)
add_dependencies(scheduler scipoptsuite_ext)

target_link_libraries(scheduler  
  ${catkin_LIBRARIES}
)

add_executable(scheduler_example src/main.cpp)
target_link_libraries(scheduler_example
  scheduler
)

add_executable(error_test src/error_test.cpp)
target_link_libraries(error_test
  scheduler
)

add_executable(scheduler_node src/scheduler_node.cpp)
add_dependencies(scheduler_node strands_executive_msgs_generate_messages_cpp)
target_link_libraries(scheduler_node
  scheduler  
  ${catkin_LIBRARIES}
)


# Specify libraries to link a library or executable target against



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

# Mark executables and/or libraries for installation
install(TARGETS scheduler_example error_test scheduler_node scheduler
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

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

if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest)

  add_rostest(tests/scheduler.test)

endif()