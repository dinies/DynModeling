// Created by Edoardo Ghini on 12/07/2018.

#define BOOST_TEST_MODULE MyMathTests

#include <boost/test/unit_test.hpp>
#include <limits>

#include "../../src/utils/MyMath.hpp"

namespace dyn_modeling{
  BOOST_AUTO_TEST_SUITE( MMath)

  BOOST_AUTO_TEST_CASE( Box_minus) {

    double res =  MyMath::boxMinusAngleRad(0.1, 0.2);
    double threshold = 0.0001;
    BOOST_CHECK_CLOSE( res, -0.1, threshold);
  }
  BOOST_AUTO_TEST_CASE( Box_plus) {
    double res =  MyMath::boxPlusAngleRad(3.14, 0.02);
    double threshold = 0.01;
    BOOST_CHECK_CLOSE( res, -3.123, threshold);
  }

  BOOST_AUTO_TEST_CASE( MatrixEigenUsage) {
    Eigen::Matrix3d affinity1;
    affinity1 << 1 , 0 , 0,
      0 , 1 , 0 ,
      0 , 0 , 1;
    Eigen::Matrix<double,3,3> affinity2;
    affinity2 << 1 , 0 , 0,
      0 , 1 , 0 ,
      0 , 0 , 1;
    Eigen::Vector3d same( 3 , 3 , 1 );

    Eigen::Vector3d res1 = affinity1 * same;
    Eigen::Vector3d res2 = affinity2 * same;

    double threshold = 0.01;
    BOOST_CHECK_SMALL( same(0) - res1(0) , threshold);
    BOOST_CHECK_SMALL( same(1) - res1(1) , threshold);
    BOOST_CHECK_SMALL( same(2) - res1(2) , threshold);
    BOOST_CHECK_SMALL( same(0) - res2(0) , threshold);
    BOOST_CHECK_SMALL( same(1) - res2(1) , threshold);
    BOOST_CHECK_SMALL( same(2) - res2(2) , threshold);
  }


  BOOST_AUTO_TEST_CASE( v2tAllZeros) {
    std::vector<double> initial_guess= { 0 , 0 , 0};
    Eigen::Isometry2d result =  MyMath::v2t(initial_guess);
    Eigen::Isometry2d truth;
    truth.setIdentity();
    Eigen::Vector2d zeros( 0, 0);
    truth.translation() = zeros;
    double threshold = 0.0001;
    BOOST_CHECK_SMALL( result(0,0) - truth(0,0) , threshold);
    BOOST_CHECK_SMALL( result(0,1) - truth(0,1) , threshold);
    BOOST_CHECK_SMALL( result(0,2) - truth(0,2) , threshold);
    BOOST_CHECK_SMALL( result(1,0) - truth(1,0) , threshold);
    BOOST_CHECK_SMALL( result(1,1) - truth(1,1) , threshold);
    BOOST_CHECK_SMALL( result(1,2) - truth(1,2) , threshold);
    BOOST_CHECK_SMALL( result(2,0) - truth(2,0) , threshold);
    BOOST_CHECK_SMALL( result(2,1) - truth(2,1) , threshold);
    BOOST_CHECK_SMALL( result(2,2) - truth(2,2) , threshold);
  }

  BOOST_AUTO_TEST_SUITE_END()

}
