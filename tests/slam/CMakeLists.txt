add_executable( TestSlam
  TestSlam.cpp
  ../../src/slam/Slam.cpp
  ../../src/slam/DatasetManager.cpp
  ../../src/slam/Robot.cpp
  ../../src/slam/ScanMatcher.cpp
  ../../src/slam/Map.cpp
  ../../src/slam/Graph.cpp
  ../../src/slam/LineMatcher.cpp
  ../../src/slam/DataAssociator.cpp
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

add_executable( TestCriticalIterations
  TestCriticalIterations.cpp
  ../../src/slam/Slam.cpp
  ../../src/slam/DatasetManager.cpp
  ../../src/slam/Robot.cpp
  ../../src/slam/ScanMatcher.cpp
  ../../src/slam/Map.cpp
  ../../src/slam/Graph.cpp
  ../../src/slam/LineMatcher.cpp
  ../../src/slam/DataAssociator.cpp
  ../../src/utils/MyMath.cpp
  ../../src/drawing/Drawer.cpp
  )

target_link_libraries(
  TestCriticalIterations
  PUBLIC Boost::unit_test_framework
  PUBLIC ${OpenCV_LIBS}
  PUBLIC Boost::filesystem
  PUBLIC Boost::system
  PUBLIC Boost::iostreams
  )
add_test( NAME TestCriticalIterations COMMAND TestCriticalIterations )


add_executable( TestLineMatcher
  TestLineMatcher.cpp
  ../../src/slam/DatasetManager.cpp
  ../../src/slam/Robot.cpp
  ../../src/slam/LineMatcher.cpp
  ../../src/utils/MyMath.cpp
  ../../src/drawing/Drawer.cpp
  )

target_link_libraries(
  TestLineMatcher
  PUBLIC Boost::unit_test_framework
  PUBLIC ${OpenCV_LIBS}
  PUBLIC Boost::filesystem
  PUBLIC Boost::system
  PUBLIC Boost::iostreams
  )
add_test( NAME TestLineMatcher COMMAND TestLineMatcher)

add_executable( TestDataAssociator
  TestDataAssociator.cpp
  ../../src/slam/DatasetManager.cpp
  ../../src/slam/Robot.cpp
  ../../src/slam/LineMatcher.cpp
  ../../src/slam/DataAssociator.cpp
  ../../src/utils/MyMath.cpp
  ../../src/drawing/Drawer.cpp
  )

target_link_libraries(
  TestDataAssociator
  PUBLIC Boost::unit_test_framework
  PUBLIC ${OpenCV_LIBS}
  PUBLIC Boost::filesystem
  PUBLIC Boost::system
  PUBLIC Boost::iostreams
  )
add_test( NAME TestDataAssociator COMMAND TestDataAssociator)


add_executable( TestCriticalAssociator
  TestCriticalAssociator.cpp
  ../../src/slam/DatasetManager.cpp
  ../../src/slam/Robot.cpp
  ../../src/slam/Map.cpp
  ../../src/slam/LineMatcher.cpp
  ../../src/slam/DataAssociator.cpp
  ../../src/utils/MyMath.cpp
  ../../src/drawing/Drawer.cpp
  )

target_link_libraries(
  TestCriticalAssociator
  PUBLIC Boost::unit_test_framework
  PUBLIC ${OpenCV_LIBS}
  PUBLIC Boost::filesystem
  PUBLIC Boost::system
  PUBLIC Boost::iostreams
  )
add_test( NAME TestCriticalAssociator COMMAND TestCriticalAssociator)


add_executable( TestDrawLines
  TestDrawLines.cpp
  ../../src/slam/DatasetManager.cpp
  ../../src/slam/Robot.cpp
  ../../src/slam/LineMatcher.cpp
  ../../src/slam/Map.cpp
  ../../src/utils/MyMath.cpp
  ../../src/drawing/Drawer.cpp
  )

target_link_libraries(
  TestDrawLines
  PUBLIC Boost::unit_test_framework
  PUBLIC ${OpenCV_LIBS}
  PUBLIC Boost::filesystem
  PUBLIC Boost::system
  PUBLIC Boost::iostreams
  )
add_test( NAME TestDrawLines COMMAND TestDrawLines)


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
