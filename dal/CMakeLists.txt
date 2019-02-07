###############################################################
######                                                   ######
######		Author: Manuel P. J. (aka. manuoso)          ######
######                                                   ######
###############################################################

# Clasify sources according to folder structure.
# This creates a nice layout for the project in visual studio
# This macro is courtesy of http://www.cmake.org/pipermail/cmake/2013-November/056336.html
macro(GroupSources curdir)
	file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}/${curdir}
		${PROJECT_SOURCE_DIR}/${curdir}/*)
	foreach(child ${children})
		if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/${curdir}/${child})
			GroupSources(${curdir}/${child})
		else()
			string(REPLACE "/" "\\" groupname ${curdir})
			source_group(${groupname} FILES ${PROJECT_SOURCE_DIR}/${curdir}/${child})
		endif()
	endforeach()
endmacro()

GroupSources(src)

#########################################
###### Loading 3rd party libraries.######
#########################################

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -O0")

set(BUILD_SHARED_LIBS OFF)

## Load Eigen
find_package(Eigen3 REQUIRED)
if(${Eigen3_FOUND})
	MESSAGE( STATUS "Eigen FOUND")
endif()

## Load djiosdk-core
find_package(DJIOSDK REQUIRED)
if(${DJIOSDK_FOUND})
	MESSAGE( STATUS "DJIOSDK FOUND")
endif()

#########################################
######			Executable		   ######
#########################################
        
# Set SOURCES variable
file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

# Set HEADERS variable
file(GLOB_RECURSE HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/dal/dal.h ${CMAKE_CURRENT_SOURCE_DIR}/include/dal/LogStatus.h ${CMAKE_CURRENT_SOURCE_DIR}/include/dal/LogTelemetry.h)
file(GLOB_RECURSE HEADERS_BC ${CMAKE_CURRENT_SOURCE_DIR}/include/dal/backends/*.h )

# Includes directories
GroupSources(src)

include(${CMAKE_SOURCE_DIR}/cmake/LibraryConfig.cmake)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_include_directories(${PROJECT_NAME} PUBLIC ${DJIOSDK_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${DJIOSDK_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC "/usr/include/eigen3")
