add_executable(drawMovingPoints  drawMovingPoints.cpp)

target_link_libraries(
        drawMovingPoints
        PUBLIC ${OpenCV_LIBS}
)
target_link_libraries(
        drawMovingPoints
        PUBLIC Boost::boost
)
target_link_libraries(
        drawMovingPoints
        PUBLIC Boost::filesystem
)
target_link_libraries(
        drawMovingPoints
        PUBLIC Boost::system
)
target_link_libraries(
        drawMovingPoints
        PUBLIC Boost::iostreams
)



add_executable( pendulumMain pendulumMain.cpp
        ../src/pendulum/Pendulum.cpp
        ../src/utils/Clock.cpp
        ../src/controllers/Controller.cpp
)
target_link_libraries(
        pendulumMain
        PUBLIC ${OpenCV_LIBS}
)
target_link_libraries(
        pendulumMain
        PUBLIC Boost::boost
)
target_link_libraries(
        pendulumMain
        PUBLIC Boost::filesystem
)
target_link_libraries(
        pendulumMain
        PUBLIC Boost::system
)
target_link_libraries(
        pendulumMain
        PUBLIC Boost::iostreams
)

