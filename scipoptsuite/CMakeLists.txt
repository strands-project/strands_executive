cmake_minimum_required(VERSION 2.8.3)

project(scipoptsuite)

find_package(catkin REQUIRED COMPONENTS)

# FILE(GLOB scipopt_libs "${CMAKE_BINARY_DIR}/strands_executive/scipoptsuite/scipoptsuite/src/scipoptsuite_ext/scip-3.0.2/lib/*.a")
SET(scipopt_libs ${CATKIN_DEVEL_PREFIX}/lib/libscip.a ${CATKIN_DEVEL_PREFIX}/lib/libobjscip.a ${CATKIN_DEVEL_PREFIX}/lib/liblpispx.a ${CATKIN_DEVEL_PREFIX}/lib/libnlpi.cppad.a ${CATKIN_DEVEL_PREFIX}/lib/libsoplex.a ${CATKIN_DEVEL_PREFIX}/lib/libzimpl.a)
MESSAGE("providing scipopt_libs from " ${CMAKE_BINARY_DIR}/strands_executive/scipoptsuite/scipoptsuite/src/scipoptsuite_ext/scip-3.0.2/lib/)
MESSAGE("scipopt_libs " ${scipopt_libs})
set(scipopt_external_libs "gmp readline zlib m")


###########
## Build ##
###########

include(ExternalProject)

if(APPLE)
   set(PATCH_FILE ${CMAKE_CURRENT_SOURCE_DIR}/scipoptsuite-3.0.2-darwin-clang.diff)
else ()
   set(PATCH_FILE ${CMAKE_CURRENT_SOURCE_DIR}/scipoptsuite-3.0.2-linux-clang.diff)
endif ()

ExternalProject_Add(
   scipoptsuite_ext   
   PREFIX scipoptsuite
   URL http://scip.zib.de/download/release/scipoptsuite-3.0.2.tgz
   URL_MD5 db5b46986d4b8ed8aa20f2ee5876093b
   UPDATE_COMMAND ""
   PATCH_COMMAND patch -p0 < ${PATCH_FILE}
   CONFIGURE_COMMAND ""
   BUILD_COMMAND make -C ${CMAKE_BINARY_DIR}/strands_executive/scipoptsuite/scipoptsuite/src/scipoptsuite_ext
   INSTALL_COMMAND make -C ${CMAKE_BINARY_DIR}/strands_executive/scipoptsuite/scipoptsuite/src/scipoptsuite_ext/scip-3.0.2 INSTALLDIR=${CATKIN_DEVEL_PREFIX} installlibs COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/create-lib-links.bash ${CATKIN_DEVEL_PREFIX}/lib
)

# need explicit unpack inorder to apply patch
ExternalProject_Add_Step(scipoptsuite_ext extract_scip
   COMMAND ${CMAKE_COMMAND} -E tar zxvf scip-3.0.2.tgz WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/strands_executive/scipoptsuite/scipoptsuite/src/scipoptsuite_ext
   DEPENDEES download
   DEPENDERS patch
)

# need explicit unpack inorder to apply patch
ExternalProject_Add_Step(scipoptsuite_ext extract_gcg
   COMMAND ${CMAKE_COMMAND} -E tar zxvf gcg-1.1.1.tgz WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/strands_executive/scipoptsuite/scipoptsuite/src/scipoptsuite_ext
   DEPENDEES download
   DEPENDERS patch
)

# need explicit unpack inorder to apply patch
ExternalProject_Add_Step(scipoptsuite_ext extract_ug
   COMMAND ${CMAKE_COMMAND} -E tar zxvf ug-0.7.2.tgz WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/strands_executive/scipoptsuite/scipoptsuite/src/scipoptsuite_ext
   DEPENDEES download
   DEPENDERS patch
)

# need explicit unpack inorder to apply patch
ExternalProject_Add_Step(scipoptsuite_ext extract_soplex
   COMMAND ${CMAKE_COMMAND} -E tar zxvf soplex-1.7.2.tgz WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/strands_executive/scipoptsuite/scipoptsuite/src/scipoptsuite_ext
   DEPENDEES download
   DEPENDERS patch
)

# need explicit unpack inorder to apply patch
ExternalProject_Add_Step(scipoptsuite_ext extract_zimpl
   COMMAND ${CMAKE_COMMAND} -E tar zxvf zimpl-3.3.1.tgz WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/strands_executive/scipoptsuite/scipoptsuite/src/scipoptsuite_ext
   DEPENDEES download
   DEPENDERS patch
)

ExternalProject_Add_Step(scipoptsuite_ext install_headers
   COMMAND make -C ${CMAKE_BINARY_DIR}/strands_executive/scipoptsuite/scipoptsuite/src/scipoptsuite_ext/scip-3.0.2 INSTALLDIR=${CATKIN_DEVEL_PREFIX}/include INCLUDEDIR=scipoptsuite installheaders 
   DEPENDEES build
   DEPENDERS install
)

# need to make this in advance to stop catkin failing
set(HEADER_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/scipoptsuite)
file(MAKE_DIRECTORY ${HEADER_DIRECTORY})


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
 INCLUDE_DIRS ${HEADER_DIRECTORY}
 LIBRARIES ${scipopt_libs} 
 # CATKIN_DEPENDS 
 # DEPENDS gmp readline zlib m 
 EXPORTED_TARGETS scipoptsuite_ext
)


