add_executable( Test test.cpp
        )
target_link_libraries(
        Test
        PUBLIC Boost::unit_test_framework
)
add_test( NAME NameTest COMMAND Test)

add_subdirectory(drawing)
add_subdirectory(pendulum)
add_subdirectory(slam)
