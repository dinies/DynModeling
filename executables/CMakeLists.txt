add_executable(drawPointMovingTest drawPointMovingTest.cpp)

target_link_libraries(
        drawPointMovingTest
        PUBLIC ${OpenCV_LIBS}
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



add_executable( pendulumTest pendulumTest.cpp
        ../src/pendulum/Pendulum.cpp
        ../src/utils/Clock.cpp
)

