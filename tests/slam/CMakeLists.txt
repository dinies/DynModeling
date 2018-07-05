add_executable( TestSlam
  TestSlam.cpp
  # ../../src/slam/Slam.cpp
  # ../../src/slam/DatasetManager.cpp
  # ../../src/slam/Robot.cpp
  # ../../src/slam/ScanMatcher.cpp
  # ../../src/slam/Map.cpp

  # check for bug  these two classes libc++abi.dylib: terminating with uncaught exception of type std::out_of_range: basic_string
  # ../../src/utils/MyMath.cpp
  # ../../src/drawing/Drawer.cpp
  )

target_link_libraries(
  TestSlam
  PUBLIC Boost::unit_test_framework
  # PUBLIC ${OpenCV_LIBS}
  )

add_test( NAME TestSlam COMMAND TestSlam)
