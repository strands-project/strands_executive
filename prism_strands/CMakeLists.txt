cmake_minimum_required(VERSION 2.8.3)
project(prism_strands)



## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS)

## This is a bit of an odd workaround for finding Java in 14.04 in automated builds
## We want to really want to use find_package, but it doesn't know about new Java installs
## Therefore we set the JAVA_HOME var to where we know the dependency should be installed.
## But then it only gives us the executables back so we need to strip out the JAVA_HOME dir to use!
## This *should* make it robust for source compile users with different installs

## this is used for a hint for java 8 on older versions of Ubuntu such as 14.04
if(NOT ENV{JAVA_HOME})
message("Did not find JAVA_HOME, so setting our own")
set(ENV{JAVA_HOME} "/usr/lib/jvm/java-8-openjdk-amd64")
endif()

find_package(Java 1.8 REQUIRED Development)

get_filename_component(JAVAC_REALPATH ${Java_JAVAC_EXECUTABLE} REALPATH)
get_filename_component(JAVAC_DIR ${JAVAC_REALPATH} DIRECTORY)

## Set the Java compile information for prism. This doesn't really need a separate step, but is useful for testing
set(JAVA_DIR "${JAVAC_DIR}/..")

message("Using JAVA_DIR as " ${JAVA_DIR})


## System dependencies are found with CMake's conventions


include(ExternalProject)


catkin_package(
	#LIBRARIES mongoclient
	#INCLUDE_DIRS include
)

ExternalProject_add(prism
	URL https://github.com/bfalacerda/prism/archive/v4.4.robots.1.tar.gz
	PREFIX "${CMAKE_CURRENT_BINARY_DIR}/prism"
	BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/prism/src/prism/prism"
	UPDATE_COMMAND ""
	CONFIGURE_COMMAND ""
	BUILD_COMMAND PATH=${JAVA_DIR}/bin:$ENV{PATH} make JAVA_DIR=${JAVA_DIR}
	INSTALL_COMMAND rsync -a ${CMAKE_CURRENT_BINARY_DIR}/prism/src/prism/prism/lib ${CMAKE_CURRENT_BINARY_DIR}/prism/src/prism/prism/classes ${CATKIN_DEVEL_PREFIX}/lib/prism
	#CONFIGURE_COMMAND "scons -C ${CMAKE_CURRENT_BINARY_DIR}/libmongocxx"
)


## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
   scripts/prism
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# Mark cpp header files for installation
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/prism
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
  #FILES_MATCHING PATTERN "*.so"
  PATTERN ".svn" EXCLUDE
)

