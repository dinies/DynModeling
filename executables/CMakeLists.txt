add_executable(drawMovingPoints  drawMovingPoints.cpp)

target_link_libraries(
  drawMovingPoints
  PUBLIC ${OpenCV_LIBS}
  PUBLIC Boost::filesystem
  PUBLIC Boost::system
  PUBLIC Boost::iostreams
  )


add_executable( pendulumMain pendulumMain.cpp
  ../src/pendulum/Pendulum.cpp
  ../src/utils/Clock.cpp
  ../src/controllers/Controller.cpp
  ../src/utils/MyMath.cpp
  )

target_link_libraries(
  pendulumMain
  PUBLIC ${OpenCV_LIBS}
  PUBLIC Boost::filesystem
  PUBLIC Boost::system
  PUBLIC Boost::iostreams
  )

add_executable( fileParsing fileParsing.cpp
  ../src/slam/DatasetManager.cpp
  )

add_executable( robotScanRetrieving robotScanRetrieving.cpp
  ../src/slam/Robot.cpp
  ../src/utils/MyMath.cpp
  ../src/slam/DatasetManager.cpp
  )
target_link_libraries(
  robotScanRetrieving
  PUBLIC ${OpenCV_LIBS}
  PUBLIC Boost::filesystem
  PUBLIC Boost::system
  PUBLIC Boost::iostreams
  )
