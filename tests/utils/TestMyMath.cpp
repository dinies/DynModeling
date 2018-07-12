// Created by Edoardo Ghini on 12/07/2018.

#define BOOST_TEST_MODULE MyMathTests

#include <boost/test/unit_test.hpp>
#include <limits>

#include "../../src/utils/MyMath.hpp"

namespace dyn_modeling{
  BOOST_AUTO_TEST_SUITE( MMath)

  BOOST_AUTO_TEST_CASE( Box_minus) {

    const std::vector<double> t_gains{ 0, 0 };
    double res =  MyMath::boxMinusAngleRad(0.1, 0.2);
    double threshold = 0.0001;
    BOOST_CHECK_CLOSE( res, -0.1, threshold);
  }

  BOOST_AUTO_TEST_SUITE_END()

}
