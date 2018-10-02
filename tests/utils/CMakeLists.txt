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
