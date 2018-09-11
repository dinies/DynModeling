CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(dyn_modeling)

# Set the output directory for the build executables and libraries
ADD_DEFINITIONS(-DUNIX)
MESSAGE(STATUS "Compiling on Unix")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

# -g used for enabling gdb  and -D for use gdb on boost tests ( it gives an error at compilation time,give up !!)
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}  -g  -Wno-ignored-attributes -std=c++11 ")


message("build type: '${CMAKE_BUILD_TYPE}'")


set_property(GLOBAL PROPERTY GLOBAL_DEPENDS_DEBUG_MODE 1)


SET(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
SET(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}")


# Find Eigen3
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
# INCLUDE_DIRECTORIES(/usr/include/eigen3)

# Find Opencv
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

FIND_PACKAGE(Boost COMPONENTS system filesystem iostreams unit_test_framework REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})


add_subdirectory( executables)

enable_testing()
add_subdirectory( tests)







