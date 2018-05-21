# requires minimum cmake version
cmake_minimum_required(VERSION 2.8.9)

# produce the cmake var PROJECT_NAME
project(gazePupil)

# mandatory use of these packages
find_package(YARP REQUIRED)
find_package(ICUBcontrib REQUIRED)

# extend the current search path used by cmake to load helpers
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

# helpers defining certain macros (e.g. "yarp_install")
include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

# specify to install in $ICUBcontrib_DIR/bin
icubcontrib_set_default_prefix()

# import math symbols from standard cmath
add_definitions(-D_USE_MATH_DEFINES)

include_directories(${YARP_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/src)
include_directories( include )

find_package( OpenCV REQUIRED )
if( OpenCV_FOUND )
list( APPEND ThirdParty_LIBS ${OpenCV_LIBS} )
    include_directories( ${OpenCV_INCLUDE_DIRS} )
endif( OpenCV_FOUND )

# follower
add_executable(${PROJECT_NAME}-main-follower ${CMAKE_SOURCE_DIR}/include/helpers.h 
		               ${CMAKE_SOURCE_DIR}/src/icub_follower.cpp
			       ${CMAKE_SOURCE_DIR}/src/helpers.cpp  
				)

target_link_libraries(${PROJECT_NAME}-main-follower ${YARP_LIBRARIES})
target_link_libraries(${PROJECT_NAME}-main-follower ${OpenCV_LIBS})
install(TARGETS ${PROJECT_NAME}-main-follower DESTINATION bin)

# leader
add_executable(${PROJECT_NAME}-main-leader  
		              	${CMAKE_SOURCE_DIR}/src/icub_leader.cpp
				${CMAKE_SOURCE_DIR}/include/helpers.h
			       	${CMAKE_SOURCE_DIR}/include/init.h
				${CMAKE_SOURCE_DIR}/include/configure.h
			    	${CMAKE_SOURCE_DIR}/include/compute.h
			       	${CMAKE_SOURCE_DIR}/src/helpers.cpp
			       	${CMAKE_SOURCE_DIR}/src/configure.cpp
			       	${CMAKE_SOURCE_DIR}/src/compute.cpp
				)

target_link_libraries(${PROJECT_NAME}-main-leader ${YARP_LIBRARIES})
target_link_libraries(${PROJECT_NAME}-main-leader ${OpenCV_LIBS})
install(TARGETS ${PROJECT_NAME}-main-leader DESTINATION bin)

# detector
add_executable(${PROJECT_NAME}-detector ${CMAKE_SOURCE_DIR}/src/detector.cpp)
target_link_libraries(${PROJECT_NAME}-detector ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME}-detector DESTINATION bin)

# generate ad-hoc project to perform "make uninstall"
icubcontrib_add_uninstall_target()

file(GLOB scripts ${CMAKE_SOURCE_DIR}/app/scripts/*.xml)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})
