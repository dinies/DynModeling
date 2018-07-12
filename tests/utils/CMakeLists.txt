add_executable( TestMyMath
  TestMyMath.cpp
  ../../src/utils/MyMath.cpp
  )
target_link_libraries(
  TestMyMath
  PUBLIC Boost::unit_test_framework
  )
add_test( NAME TestMyMath COMMAND TestMyMath)
