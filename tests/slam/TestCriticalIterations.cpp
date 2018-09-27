// Created by dinies on 26/09/2018.


#define BOOST_TEST_MODULE SlamTests

#include <iostream>
#include <boost/test/unit_test.hpp>

#include "src/slam/Slam.hpp"
#include "opencv2/opencv.hpp"

namespace dyn_modeling{
  BOOST_AUTO_TEST_SUITE( SlamClass)

  BOOST_AUTO_TEST_CASE(criticalIterations) {
    std::string relativePath= "../files/datasets/criticalIterations.txt";
    Eigen::Vector3d initial_state(0,0,0);
    const double maxDistBetweenRanges= 3;
    const double maxAngularCoeff = 1;
    const double minLengthLines = 0.1;
    Slam slam = Slam(relativePath, initial_state, maxDistBetweenRanges, maxAngularCoeff, minLengthLines );
    slam.cycle();
    cv::waitKey();
  }
  BOOST_AUTO_TEST_SUITE_END()
}
