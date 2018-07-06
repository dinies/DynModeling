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
  )

add_executable( slamBehaviour slamBehaviour.cpp
  ../src/slam/Slam.cpp
  ../src/slam/Robot.cpp
  ../src/drawing/Drawer.cpp
  ../src/slam/Map.cpp
  ../src/slam/DatasetManager.cpp
  ../src/slam/ScanMatcher.cpp
  ../src/utils/MyMath.cpp
  )

target_link_libraries(
  slamBehaviour
  PUBLIC ${OpenCV_LIBS}
  )

add_executable( drawerTrial drawerTrial.cpp
  ../src/drawing/Drawer.cpp
  )

target_link_libraries(
  drawerTrial
  PUBLIC ${OpenCV_LIBS}
  )
