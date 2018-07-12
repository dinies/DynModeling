add_executable( TestSlam
  TestSlam.cpp
  ../../src/slam/Slam.cpp
  ../../src/slam/DatasetManager.cpp
  ../../src/slam/Robot.cpp
  ../../src/slam/ScanMatcher.cpp
  ../../src/slam/Map.cpp
  ../../src/utils/MyMath.cpp
  ../../src/drawing/Drawer.cpp
  )

target_link_libraries(
  TestSlam
  PUBLIC Boost::unit_test_framework
  PUBLIC ${OpenCV_LIBS}
  PUBLIC Boost::filesystem
  PUBLIC Boost::system
  PUBLIC Boost::iostreams
  )
add_test( NAME TestSlam COMMAND TestSlam)


add_executable( TestMap
  TestMap.cpp
  ../../src/slam/Map.cpp
  ../../src/utils/MyMath.cpp
  ../../src/drawing/Drawer.cpp
  )
target_link_libraries(
  TestMap
  PUBLIC Boost::unit_test_framework
  PUBLIC ${OpenCV_LIBS}
  )
add_test( NAME TestMap COMMAND TestMap)



add_executable( TestScanMatcher
  TestScanMatcher.cpp
  ../../src/slam/ScanMatcher.cpp
  ../../src/slam/DatasetManager.cpp
  ../../src/slam/Robot.cpp
  ../../src/utils/MyMath.cpp
  )

target_link_libraries(
  TestScanMatcher
  PUBLIC Boost::unit_test_framework
  PUBLIC ${OpenCV_LIBS}
  PUBLIC Boost::filesystem
  PUBLIC Boost::system
  PUBLIC Boost::iostreams
  )
add_test( NAME TestScanMatcher COMMAND TestScanMatcher)


add_executable( TestRobot
  TestRobot.cpp
  ../../src/slam/Robot.cpp
  ../../src/slam/DatasetManager.cpp
  ../../src/utils/MyMath.cpp
  )

target_link_libraries(
  TestRobot
  PUBLIC Boost::unit_test_framework
  PUBLIC ${OpenCV_LIBS}
  PUBLIC Boost::filesystem
  PUBLIC Boost::system
  PUBLIC Boost::iostreams
  )
add_test( NAME TestRobot COMMAND TestRobot)


add_executable( TestDatasetManager
  TestDatasetManager.cpp
  ../../src/slam/DatasetManager.cpp
  )

target_link_libraries(
  TestDatasetManager
  PUBLIC Boost::unit_test_framework
  )
add_test( NAME TestDatasetManager COMMAND TestDatasetManager)
