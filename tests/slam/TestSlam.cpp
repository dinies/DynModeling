// Created by dinies on 26/06/2018.


#define BOOST_TEST_MODULE SlamTests

#include <iostream>
#include <boost/test/unit_test.hpp>

#include "src/slam/Slam.hpp"
#include "opencv2/opencv.hpp"

namespace dyn_modeling{
  BOOST_AUTO_TEST_SUITE( SlamClass)

  BOOST_AUTO_TEST_CASE(behaviouralTest) {
    // std::string relativePath= "../files/datasets/first100data.txt";
    std::string relativePath= "../files/datasets/realLaserScans.txt";
    Eigen::Vector3d initial_state( 0,0,0);
    const double maxDistBetweenSPoints = 5;
    const double maxAngularCoeff = 1.4;
    Slam slam = Slam(relativePath, initial_state, maxDistBetweenSPoints, maxAngularCoeff );
    slam.cycle();
    cv::waitKey();
  }
  BOOST_AUTO_TEST_SUITE_END()
}
