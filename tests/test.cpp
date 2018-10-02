// Created by Edoardo Ghini on 20/06/2018.

#define BOOST_TEST_MODULE Simple testcases

#include <boost/test/unit_test.hpp>
#include <iostream>
#include <boost/filesystem.hpp>

BOOST_AUTO_TEST_CASE( simple_test) {
    BOOST_CHECK_EQUAL( 2+2, 4);
}

BOOST_AUTO_TEST_CASE( path_test) {
    boost::filesystem::path full_path(boost::filesystem::current_path());
    std::cout << "Current path is : " << full_path << std::endl;
}

