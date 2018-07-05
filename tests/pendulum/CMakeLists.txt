add_executable( TestController TestController.cpp ../../src/controllers/Controller.cpp
  )
target_link_libraries(
  TestController
  PUBLIC Boost::unit_test_framework
  )
add_test( NAME TestController COMMAND TestController)
