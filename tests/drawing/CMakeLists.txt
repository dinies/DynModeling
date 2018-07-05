add_executable( TestDrawer
  TestDrawer.cpp
  ../../src/drawing/Drawer.cpp
  )

target_link_libraries(
  TestDrawer
  PUBLIC Boost::unit_test_framework
  PUBLIC ${OpenCV_LIBS}
  )

add_test( NAME TestDrawer COMMAND TestDrawer)

