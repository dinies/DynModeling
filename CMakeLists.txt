CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(dyn_modeling)

# Set the output directory for the build executables and libraries
ADD_DEFINITIONS(-DUNIX)
MESSAGE(STATUS "Compiling on Unix")

SET(CMAKE_CXX_FLAGS_DEBUG "-Wall -Wextra -std=c++11") #ds enable all warnings and c++11 features
#SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG") #ds enable maximum optimization and disable debug symbols for cpp
#SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -DNDEBUG") #ds enable maximum optimization and disable debug symbols for c
#SET(CMAKE_BUILD_TYPE Release)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
message("build type: '${CMAKE_BUILD_TYPE}'")



SET(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
SET(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}")


#add_subdirectory(bin)
# Find Eigen3
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
#INCLUDE_DIRECTORIES(/usr/include/eigen3)

# Find Opencv
FIND_PACKAGE(OpenCV  REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

#FIND_PACKAGE(Boost  REQUIRED COMPONENTS filesystem system iostreams )
#SET( BOOST_ROOT /usr/local/Cellar/boost/1.67.0_1/include/boost)
#FIND_PACKAGE(Boost  REQUIRED )
#INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
#SET(BOOST_SOURCE /usr/local/include/boost)
add_subdirectory( boost-cmake)


message([AUTHOR_WARNING] "ciaaooaoaoaoaoaoa")
add_executable(drawPointMovingTest src/drawPointMovingTest.cpp)

target_link_libraries(
        drawPointMovingTest
        ${OpenCV_LIBS}
#        ${Boost_INCLUDE_DIRS}
#        Boost::system
#        Boost::filesystem
#        Boost::iostreams
#        PUBLIC Boost::filesystem
#        PUBLIC Boost::system
#        PUBLIC Boost::iostreams
)
target_link_libraries(
        drawPointMovingTest
        PUBLIC Boost::boost
        )
target_link_libraries(
        drawPointMovingTest
        PUBLIC Boost::filesystem
        )
target_link_libraries(
        drawPointMovingTest
        PUBLIC Boost::system
        )
target_link_libraries(
        drawPointMovingTest
        PUBLIC Boost::iostreams
        )
message(STATUS $ENV{PATH})






