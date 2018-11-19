add_executable( TestMyMath
  TestMyMath.cpp
  ../../src/utils/MyMath.cpp
  )
target_link_libraries(
  TestMyMath
  PUBLIC Boost::unit_test_framework
  )
add_test( NAME TestMyMath COMMAND TestMyMath)

add_executable( TestDrawEllipse
  TestEllipse.cpp
  ../../src/utils/Ellipse.cpp
  )
target_link_libraries(
  TestDrawEllipse
  PUBLIC ${OpenCV_LIBS}
  PUBLIC Boost::unit_test_framework
  )

add_test( NAME TestDrawEllipse COMMAND TestDrawEllipse)


# TODO  remove this, since we want to have a test runner that runs all tests !
 #  add_executable(
 # TestClock
 # TestClock.cpp
 # )


 # target_link_libraries(
 # TestClock
 # gmock_main
 # )

 # legacy mode  
 # target_include_directories(
 # TestClock
 # PUBLIC "${gtest_SOURCE_DIR}/include" "${gmock_SOURCE_DIR}/include"
 # )



