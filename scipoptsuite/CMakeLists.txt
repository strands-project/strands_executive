cmake_minimum_required(VERSION 2.8.3)

project(scipoptsuite)

find_package(catkin REQUIRED COMPONENTS
)

MESSAGE("CMAKE_BINARY_DIR " ${CMAKE_BINARY_DIR})
MESSAGE("providing scipopt_libs from " ${CMAKE_BINARY_DIR}/scipoptsuite/scipoptsuite_ext-prefix/src/scipoptsuite_ext/scip-3.0.2/lib/)
MESSAGE("scipopt_libs " ${scipopt_libs})


find_library(GMP_LIB gmp)
find_library(Z_LIB z)
find_library(M_LIB m)
find_library(NCURSES_LIB ncurses)

###########
## Build ##
###########

include(ExternalProject)

if(APPLE)
   set(PATCH_FILE ${CMAKE_CURRENT_SOURCE_DIR}/scipoptsuite-3.0.2-darwin-clang.diff)
   set(SHARED_OBJS false)
   SET(scipopt_devel_lib_files ${CATKIN_DEVEL_PREFIX}/lib/libscip.a ${CATKIN_DEVEL_PREFIX}/lib/libobjscip.a ${CATKIN_DEVEL_PREFIX}/lib/liblpispx.a ${CATKIN_DEVEL_PREFIX}/lib/libnlpi.cppad.a ${CATKIN_DEVEL_PREFIX}/lib/libsoplex.a ${CATKIN_DEVEL_PREFIX}/lib/libzimpl.a)
  else ()
   set(PATCH_FILE ${CMAKE_CURRENT_SOURCE_DIR}/scipoptsuite-3.0.2-linux-clang.diff)
   set(SHARED_OBJS true)
   SET(scipopt_devel_lib_files ${CATKIN_DEVEL_PREFIX}/lib/libscip.so ${CATKIN_DEVEL_PREFIX}/lib/libobjscip.so ${CATKIN_DEVEL_PREFIX}/lib/liblpispx.so ${CATKIN_DEVEL_PREFIX}/lib/libnlpi.cppad.so ${CATKIN_DEVEL_PREFIX}/lib/libsoplex.so ${CATKIN_DEVEL_PREFIX}/lib/libzimpl.so)
endif ()

# MESSAGE("CMAKE_BINARY_DIR " ${CMAKE_BINARY_DIR})
# MESSAGE("CATKIN_DEVEL_PREFIX " ${CATKIN_DEVEL_PREFIX})
# MESSAGE("providing scipopt_libs from " ${scipoptsuite_BINARY_DIR}/scipoptsuite_ext-prefix/src/scipoptsuite_ext/scip-3.0.2/lib/)
# MESSAGE("scipopt_libs " ${scipopt_libs})

ExternalProject_Add(
   scipoptsuite_ext   
   URL http://scip.zib.de/download/release/scipoptsuite-3.0.2.tgz
   URL_MD5 db5b46986d4b8ed8aa20f2ee5876093b
   UPDATE_COMMAND ""
   PATCH_COMMAND patch -p0 < ${PATCH_FILE}
   CONFIGURE_COMMAND ""
   BUILD_COMMAND make -j4 SHARED=${SHARED_OBJS} READLINE=false -C ${scipoptsuite_BINARY_DIR}/scipoptsuite_ext-prefix/src/scipoptsuite_ext
   INSTALL_COMMAND make SHARED=${SHARED_OBJS} READLINE=false -C ${scipoptsuite_BINARY_DIR}/scipoptsuite_ext-prefix/src/scipoptsuite_ext/scip-3.0.2 SHARED=${SHARED_OBJS} INSTALLDIR=${CATKIN_DEVEL_PREFIX} installlibs COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/create-lib-links.bash ${CATKIN_DEVEL_PREFIX}/lib
)


# need explicit unpack inorder to apply patch
ExternalProject_Add_Step(scipoptsuite_ext extract_scip                                       scipoptsuite/strands_executive/scipoptsuite/src/scipoptsuite_ext
   COMMAND ${CMAKE_COMMAND} -E tar zxvf scip-3.0.2.tgz WORKING_DIRECTORY ${scipoptsuite_BINARY_DIR}/scipoptsuite_ext-prefix/src/scipoptsuite_ext
   DEPENDEES download
   DEPENDERS patch
)

# need explicit unpack inorder to apply patch
ExternalProject_Add_Step(scipoptsuite_ext extract_gcg
   COMMAND ${CMAKE_COMMAND} -E tar zxvf gcg-1.1.1.tgz WORKING_DIRECTORY ${scipoptsuite_BINARY_DIR}/scipoptsuite_ext-prefix/src/scipoptsuite_ext
   DEPENDEES download
   DEPENDERS patch
)

# need explicit unpack inorder to apply patch
ExternalProject_Add_Step(scipoptsuite_ext extract_ug
   COMMAND ${CMAKE_COMMAND} -E tar zxvf ug-0.7.2.tgz WORKING_DIRECTORY ${scipoptsuite_BINARY_DIR}/scipoptsuite_ext-prefix/src/scipoptsuite_ext
   DEPENDEES download
   DEPENDERS patch
)

# need explicit unpack inorder to apply patch
ExternalProject_Add_Step(scipoptsuite_ext extract_soplex
   COMMAND ${CMAKE_COMMAND} -E tar zxvf soplex-1.7.2.tgz WORKING_DIRECTORY ${scipoptsuite_BINARY_DIR}/scipoptsuite_ext-prefix/src/scipoptsuite_ext
   DEPENDEES download
   DEPENDERS patch
)

# need explicit unpack inorder to apply patch
ExternalProject_Add_Step(scipoptsuite_ext extract_zimpl
   COMMAND ${CMAKE_COMMAND} -E tar zxvf zimpl-3.3.1.tgz WORKING_DIRECTORY ${scipoptsuite_BINARY_DIR}/scipoptsuite_ext-prefix/src/scipoptsuite_ext
   DEPENDEES download
   DEPENDERS patch
)

ExternalProject_Add_Step(scipoptsuite_ext install_headers
   COMMAND make -C ${scipoptsuite_BINARY_DIR}/scipoptsuite_ext-prefix/src/scipoptsuite_ext/scip-3.0.2 INSTALLDIR=${CATKIN_DEVEL_PREFIX}/include INCLUDEDIR=scipoptsuite installheaders 
   DEPENDEES build
   DEPENDERS install
)

# need to make this in advance to stop catkin failing
set(HEADER_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/scipoptsuite)
# file(MAKE_DIRECTORY ${HEADER_DIRECTORY})



# make a fake include dir just to keep catkin happy. cfg_extras adds the includes proper
file(MAKE_DIRECTORY include)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${GMP_LIB} ${Z_LIB} ${M_LIB} ${NCURSES_LIB} 
 EXPORTED_TARGETS scipoptsuite_ext 
 CFG_EXTRAS scipoptsuite-extras.cmake
)


# Mark cpp header files for installation
install(DIRECTORY ${HEADER_DIRECTORY}/blockmemshell ${HEADER_DIRECTORY}/nlpi ${HEADER_DIRECTORY}/objscip ${HEADER_DIRECTORY}/scip
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


## Mark executables and/or libraries for installation



#FILE(GLOB install_these_libs ${CATKIN_DEVEL_PREFIX}/lib/lib*.so)
#FILE( GLOB install_these_libs "/usr/lib/*.so" )

# find the build-time files to install using this magic...
install(
    CODE "
    FILE( GLOB install_these_libs \"${CATKIN_DEVEL_PREFIX}/lib/*.so\" )
    FILE(INSTALL \${install_these_libs} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
    "
)