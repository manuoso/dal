##---------------------------------------------------------------------------------------------------------------------
##  DJI ABSTRACTION LAYER
##---------------------------------------------------------------------------------------------------------------------
##  Copyright 2019 Manuel Pérez Jiménez (a.k.a. manuoso) manuperezj@gmail.com
##---------------------------------------------------------------------------------------------------------------------
##  Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
##  and associated documentation files (the "Software"), to deal in the Software without restriction, 
##  including without limitation the rights to use, copy, modify, merge, publish, distribute, 
##  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
##  furnished to do so, subject to the following conditions:
##
##  The above copyright notice and this permission notice shall be included in all copies or substantial 
##  portions of the Software.
##
##  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
##  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
##  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES 
##  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
##  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
##---------------------------------------------------------------------------------------------------------------------

cmake_minimum_required (VERSION 3.14 FATAL_ERROR)
project(dal VERSION 0.1 LANGUAGES C CXX)

##########################################
######		   Configure project		  ######
##########################################

# Some build options
option(BUILD_TOOLS "Build tools module" OFF)
option(BUILD_EXAMPLES "Build example module" OFF)

option(BUILD_PYTHON "Build python module" OFF)

#########################################
###### Loading 3rd party libraries ######
#########################################

if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release CACHE STRING
       "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
       FORCE )
endif()

# set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules/" ${CMAKE_MODULE_PATH})

list(APPEND DAL_DEFINITIONS "")
set(DAL_COMPILE_OPTIONS "-Wall -g -O0")

#########################################
## Load Eigen
find_package(Eigen3 REQUIRED)
if(${Eigen3_FOUND})
	MESSAGE( STATUS "Eigen FOUND")
	list(APPEND DAL_DEFINITIONS "DAL_USE_EIGEN3")
endif()

#########################################
## Load DJIOSDK
find_package(DJIOSDK REQUIRED)
if(${DJIOSDK_FOUND})
	MESSAGE( STATUS "DJIOSDK FOUND")
	list(APPEND DAL_DEFINITIONS "DAL_USE_DJIOSDK")
endif()

#########################################
######			     Library        	 ######
#########################################

# Collect all sources
file(GLOB_RECURSE DAL_SOURCE_FILES "src/*.c" "src/*.cpp")
file(GLOB_RECURSE DAL_HEADER_FILES "include/*.inl" "include/*.h")

add_library(${PROJECT_NAME} SHARED ${DAL_SOURCE_FILES} ${DAL_HEADER_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include) 

target_compile_definitions(${PROJECT_NAME} PUBLIC ${DAL_DEFINITIONS})
string(REPLACE " " ";" DAL_COMPILE_OPTIONS_REPLACED_FLAGS ${DAL_COMPILE_OPTIONS})

target_compile_options(${PROJECT_NAME} PUBLIC ${DAL_COMPILE_OPTIONS_REPLACED_FLAGS})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)

#########################################

target_include_directories(${PROJECT_NAME} PUBLIC "/usr/include/eigen3")

target_include_directories(${PROJECT_NAME} PUBLIC ${DJIOSDK_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${DJIOSDK_LIBRARIES})

#########################################
######			Documentation	   ######
#########################################

# find_package(Doxygen)
# if(DOXYGEN_FOUND)
# 	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
# 	add_custom_target(	doc_rgbd_tools
# 						${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
# 						WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
# 						COMMENT "Generating API documentation with Doxygen" VERBATIM )
# endif(DOXYGEN_FOUND)

#########################################
######			     Tools  		       ######
#########################################

if(${BUILD_TOOLS})
	add_subdirectory(tools)
endif(${BUILD_TOOLS})

#########################################
######			    Examples 		       ######
#########################################

if(${BUILD_EXAMPLES})
	add_subdirectory(examples)
endif(${BUILD_EXAMPLES})

#########################################
######			    Python   		       ######
#########################################

if(${BUILD_PYTHON})
	add_subdirectory(python)
endif(${BUILD_PYTHON})

#########################################
######			     Install 		       ######
#########################################

include(CMakePackageConfigHelpers)

if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif() 

install(TARGETS dal
        EXPORT dalTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION ${CMAKE_INSTALL_PREFIX}
        )

write_basic_package_version_file(
		dalConfigVersion.cmake
    VERSION ${PACKAGE_VERSION}
    COMPATIBILITY AnyNewerVersion
    )

# This installs the include folder
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX} FILES_MATCHING PATTERN "*.h")
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX} FILES_MATCHING PATTERN "*.inl")

export(TARGETS dal NAMESPACE dal:: FILE dalTargets.cmake)

# This generates dalTargets.cmake
install(EXPORT dalTargets
    FILE dalTargets.cmake
    NAMESPACE dal::
    DESTINATION lib/cmake/dal
    )

configure_file(dalConfig.cmake.in dalConfig.cmake @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/dalConfig.cmake"
        DESTINATION lib/cmake/dal
        )

# uninstall target 
if(NOT TARGET uninstall) 
  configure_file( 
      "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in" 
      "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" 
      IMMEDIATE @ONLY) 
 
  add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake) 
endif() 
