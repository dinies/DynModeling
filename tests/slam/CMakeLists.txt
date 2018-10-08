add_executable( TestSlam
  TestSlam.cpp
  )

target_link_libraries(
  TestSlam
  PUBLIC slamSources
  PUBLIC Boost::unit_test_framework
  PUBLIC ${OpenCV_LIBS}
  PUBLIC Boost::filesystem
  PUBLIC Boost::system
  PUBLIC Boost::iostreams
  )
add_test( NAME TestSlam COMMAND TestSlam)

add_executable( TestCriticalIterations
  TestCriticalIterations.cpp
)

target_link_libraries(
  TestCriticalIterations
  PUBLIC slamSources
  PUBLIC Boost::unit_test_framework
  PUBLIC ${OpenCV_LIBS}
  PUBLIC Boost::filesystem
  PUBLIC Boost::system
  PUBLIC Boost::iostreams
  )
add_test( NAME TestCriticalIterations COMMAND TestCriticalIterations )


add_executable( TestGraph
  TestGraph.cpp
  )

target_link_libraries(
  TestGraph
  PUBLIC slamSources
  PUBLIC Boost::unit_test_framework
  PUBLIC ${OpenCV_LIBS}
  PUBLIC Boost::filesystem
  PUBLIC Boost::system
  PUBLIC Boost::iostreams
  )
add_test( NAME TestGraph COMMAND TestGraph)


add_executable( TestLineMatcher
  TestLineMatcher.cpp
 )

target_link_libraries(
  TestLineMatcher
  PUBLIC slamSources
  PUBLIC Boost::unit_test_framework
  PUBLIC ${OpenCV_LIBS}
  PUBLIC Boost::filesystem
  PUBLIC Boost::system
  PUBLIC Boost::iostreams
  )
add_test( NAME TestLineMatcher COMMAND TestLineMatcher)

add_executable( TestDataAssociator
  TestDataAssociator.cpp
 )

target_link_libraries(
  TestDataAssociator
  PUBLIC slamSources
  PUBLIC Boost::unit_test_framework
  PUBLIC ${OpenCV_LIBS}
  PUBLIC Boost::filesystem
  PUBLIC Boost::system
  PUBLIC Boost::iostreams
  )
add_test( NAME TestDataAssociator COMMAND TestDataAssociator)


add_executable( TestCriticalAssociator
  TestCriticalAssociator.cpp
 )

target_link_libraries(
  TestCriticalAssociator
  PUBLIC slamSources
  PUBLIC Boost::unit_test_framework
  PUBLIC ${OpenCV_LIBS}
  PUBLIC Boost::filesystem
  PUBLIC Boost::system
  PUBLIC Boost::iostreams
  )
add_test( NAME TestCriticalAssociator COMMAND TestCriticalAssociator)

add_executable( TestDrawLines
  TestDrawLines.cpp
 )

target_link_libraries(
  TestDrawLines
  PUBLIC slamSources
  PUBLIC Boost::unit_test_framework
  PUBLIC ${OpenCV_LIBS}
  PUBLIC Boost::filesystem
  PUBLIC Boost::system
  PUBLIC Boost::iostreams
  )
add_test( NAME TestDrawLines COMMAND TestDrawLines)


add_executable( TestTuningLineMatcherParams
  TestTuningLineMatcherParams.cpp
 )

target_link_libraries(
  TestTuningLineMatcherParams
  PUBLIC slamSources
  PUBLIC Boost::unit_test_framework
  PUBLIC ${OpenCV_LIBS}
  PUBLIC Boost::filesystem
  PUBLIC Boost::system
  PUBLIC Boost::iostreams
  )
add_test( NAME TestTuningLineMatcherParams COMMAND TestTuningLineMatcherParams)


add_executable( TestMap
  TestMap.cpp
 )
target_link_libraries(
  TestMap
  PUBLIC slamSources
  PUBLIC Boost::unit_test_framework
  PUBLIC ${OpenCV_LIBS}
  )
add_test( NAME TestMap COMMAND TestMap)


add_executable( TestScanMatcher
  TestScanMatcher.cpp
 )

target_link_libraries(
  TestScanMatcher
  PUBLIC slamSources
  PUBLIC Boost::unit_test_framework
  PUBLIC ${OpenCV_LIBS}
  PUBLIC Boost::filesystem
  PUBLIC Boost::system
  PUBLIC Boost::iostreams
  )
add_test( NAME TestScanMatcher COMMAND TestScanMatcher)


add_executable( TestRobot
  TestRobot.cpp
 )

target_link_libraries(
  TestRobot
  PUBLIC slamSources
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
