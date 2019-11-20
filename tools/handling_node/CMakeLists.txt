##---------------------------------------------------------------------------------------------------------------------
##  Handling node
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
project(handling_node)

#########################################
###### Loading 3rd party libraries.######
#########################################

#########################################
## Load Fastcom
find_package(fastcom REQUIRED )
if(${fastcom_FOUND})
	MESSAGE( STATUS "fastcom FOUND")
endif()

#########################################
## Load rapidjson
find_package(RapidJSON REQUIRED)

#########################################
## Load Eigen
find_package(Eigen3 REQUIRED)
if(${Eigen3_FOUND})
	MESSAGE( STATUS "Eigen FOUND")
endif()

#########################################
## Compiler specific options
set(HANDLING_NODE_COMPILE_OPTIONS "-Wall -g -O0")

## Collect all sources
file(GLOB_RECURSE SRC_FILES "src/*.cpp" "include/*.h" "include/*.inl" )

## Add Executable
add_executable(${PROJECT_NAME} ${SRC_FILES})
include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include)

string(REPLACE " " ";" HANDLING_NODE_COMPILE_OPTIONS_REPLACED_FLAGS ${HANDLING_NODE_COMPILE_OPTIONS})

target_compile_options(${PROJECT_NAME} PUBLIC ${HANDLING_NODE_COMPILE_OPTIONS_REPLACED_FLAGS})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)

target_link_libraries(${PROJECT_NAME} LINK_PUBLIC fastcom::fastcom)

target_include_directories(${PROJECT_NAME} PRIVATE ${DAL_LIBRARY_PATH}/include)
target_link_libraries(${PROJECT_NAME} LINK_PRIVATE dal)

target_include_directories(${PROJECT_NAME} PUBLIC "/usr/include/eigen3")

target_include_directories(${PROJECT_NAME} PUBLIC ${RAPIDJSON_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${RAPIDJSON_LIBRARIES})
