add_definitions(-DBOOST_TEST_DYN_LINK)

add_executable( Test test.cpp
        )
target_link_libraries(
        Test
        PUBLIC Boost::unit_test_framework
        PUBLIC Boost::filesystem
)
add_test( NAME NameTest COMMAND Test)


add_executable(
  TestGoogleTest
  TestGoogleTest.cpp
  )
target_link_libraries(
  TestGoogleTest
  gtest_main
  )

add_test(
  NAME TestGoogleTest
  COMMAND TestGoogleTest
  )


add_executable(
  TestRunner
  TestRunner.cpp
  ./slam/TestLoopCloser.cpp
  ./kdtree/TestKdTree.cpp
  ./utils/TestClock.cpp
  )
target_link_libraries(
  TestRunner
  PUBLIC gtest_main
  PUBLIC gmock_main
  PUBLIC slamSources
  PUBLIC ${OpenCV_LIBS}
  PUBLIC Boost::filesystem
  PUBLIC Boost::system
  PUBLIC Boost::iostreams
 )

add_test(
  NAME TestRunner
  COMMAND TestRunner
  )


add_subdirectory(drawing)
add_subdirectory(pendulum)
add_subdirectory(slam)
add_subdirectory(utils)

