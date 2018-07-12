// Created by Edoardo Ghini on 20/06/2018.

#define BOOST_TEST_MODULE ControllerTests

#include <boost/test/unit_test.hpp>
#include <limits>

#include "../../src/controllers/Controller.hpp"

namespace dyn_modeling{
  BOOST_AUTO_TEST_SUITE( Co)


  BOOST_AUTO_TEST_CASE( Dummy) {
    BOOST_CHECK_EQUAL(0, 0);
  }
  BOOST_AUTO_TEST_SUITE_END()
}
