// Created by Edoardo Ghini on 20/06/2018.

#define BOOST_TEST_MODULE ControllerTests

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE( Controller)


BOOST_AUTO_TEST_CASE( simple_test) {
    BOOST_CHECK_EQUAL( 2+2, 4);
}

BOOST_AUTO_TEST_CASE( simple_test2) {
    BOOST_CHECK_EQUAL( 2+2, 4);
}

BOOST_AUTO_TEST_SUITE_END()
