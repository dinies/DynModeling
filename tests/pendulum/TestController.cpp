// Created by Edoardo Ghini on 20/06/2018.

#define BOOST_TEST_MODULE ControllerTests

#include <boost/test/unit_test.hpp>
#include "../../src/controllers/Controller.hpp"

namespace dyn_modeling{
BOOST_AUTO_TEST_SUITE( C1)


BOOST_AUTO_TEST_CASE( Box_minus) {

  const std::vector<double> t_gains{ 0, 0 };
  Controller c( t_gains);
  double res =  c.boxMinusAngleRad(0.1, 0.2);
  BOOST_CHECK_EQUAL( std::round(10*res)/10, -0.1);
}

BOOST_AUTO_TEST_CASE( simple_test2) {
  BOOST_CHECK_EQUAL( 2+2, 4);
}

BOOST_AUTO_TEST_SUITE_END()

}
