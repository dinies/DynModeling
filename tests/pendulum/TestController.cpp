// Created by Edoardo Ghini on 20/06/2018.

#define BOOST_TEST_MODULE ControllerTests

#include <boost/test/unit_test.hpp>
#include <limits>

#include "../../src/controllers/Controller.hpp"

namespace dyn_modeling{
BOOST_AUTO_TEST_SUITE( C1)


BOOST_AUTO_TEST_CASE( Box_minus) {

  const std::vector<double> t_gains{ 0, 0 };
  Controller c( t_gains);
  double res =  c.boxMinusAngleRad(0.1, 0.2);
  double threshold = 0.0001;
  BOOST_CHECK_CLOSE( res, -0.1, threshold);
}

BOOST_AUTO_TEST_SUITE_END()

}
