cmake_minimum_required(VERSION 3.14)
project(dalpy VERSION 0.1)

message(STATUS "CMAKE_CURRENT_SOURCE_DIR = ${CMAKE_CURRENT_SOURCE_DIR}")

find_package(Boost COMPONENTS python3)
find_package(PythonInterp 3)
find_package(PythonLibs 3 REQUIRED)

message(STATUS "PYTHON_LIBRARIES = ${PYTHON_LIBRARIES}")
message(STATUS "PYTHON_EXECUTABLE = ${PYTHON_EXECUTABLE}")
message(STATUS "PYTHON_INCLUDE_DIRS = ${PYTHON_INCLUDE_DIRS}")

execute_process(
  COMMAND "${PYTHON_EXECUTABLE}" -c "if True:
    from distutils import sysconfig as sc
    print(sc.get_python_lib())"
  OUTPUT_VARIABLE PYTHON_SITE
  OUTPUT_STRIP_TRAILING_WHITESPACE
  )

message(STATUS "PYTHON_SITE = ${PYTHON_SITE}")

message(STATUS "Boost_LIBRARIES = ${Boost_LIBRARIES}")

list(APPEND DALPY_DEFINITIONS "")

#########################################
file(GLOB_RECURSE SRC_FILES "src/*.cpp")

add_library(${PROJECT_NAME} SHARED ${SRC_FILES})

target_compile_definitions(${PROJECT_NAME} PUBLIC ${DALPY_DEFINITIONS})

target_include_directories(${PROJECT_NAME} PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${Boost_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC ${PYTHON_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${PYTHON_LIBRARIES})

target_include_directories(${PROJECT_NAME} PRIVATE ${DAL_LIBRARY_PATH}/include)
target_link_libraries(${PROJECT_NAME} LINK_PRIVATE dal)

# Tweaks the name of the library to match what Python expects
# set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX .pyd)
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")

#########################################
######			INSTALL 		   ######
#########################################
include(CMakePackageConfigHelpers)

set(CMAKE_INSTALL_PREFIX ${PYTHON_SITE})

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}
        )
