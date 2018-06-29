add_executable( TestSlam
  TestSlam.cpp
  ../../src/slam/Slam.cpp
  ../../src/slam/DatasetManager.cpp
        )
target_link_libraries(
        TestSlam
        PUBLIC Boost::unit_test_framework
)
add_test( NAME TestSlam COMMAND TestSlam)
