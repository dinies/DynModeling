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


add_subdirectory(drawing)
add_subdirectory(pendulum)
add_subdirectory(slam)
add_subdirectory(utils)
