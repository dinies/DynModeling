add_executable( TestSlam
  TestSlam.cpp
  ../../src/slam/Slam.cpp
  ../../src/slam/DatasetManager.cpp
  ../../src/slam/Robot.cpp
  ../../src/slam/ScanMatcher.cpp
  ../../src/slam/Map.cpp
        )
target_link_libraries(
        TestSlam
        PUBLIC Boost::unit_test_framework
)
add_test( NAME TestSlam COMMAND TestSlam)
