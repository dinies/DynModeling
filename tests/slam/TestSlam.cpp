// Created by dinies on 26/06/2018.


#define BOOST_TEST_MODULE SlamTests

#include <iostream>
#include <boost/test/unit_test.hpp>

#include "src/slam/Slam.hpp"
#include "opencv2/opencv.hpp"

namespace dyn_modeling{
  BOOST_AUTO_TEST_SUITE( SlamClass
)

  BOOST_AUTO_TEST_CASE(behaviouralTest) {

    std::string relativePath= "../files/datasets/first100data.txt";
    // std::string relativePath= "../files/datasets/realLaserScans.txt";
    Eigen::Vector3d initial_state(0,0,0);

    paramsSlam params = paramsSlam( 6, 0.4, 0.6, 1, 0.1, 6, 0.1, 0.3, 3, 0.05);
    //1 int icpIterationsCap;
    //2 double kernelThresholdScanMatching;
    //3 double maxDistBetweenRangesLineMatcher;
    //4 double maxAngularCoeffLineMatcher;
    //5 double minLengthLinesLineMatcher;
    //6 int maxCandidatesAssociation;
    //7 double maxLengthDiffAssociation;
    //8 double maxOrientationDiffAssociation;
    //9 int numMiddleScanPoints;
    //10 double borderRatio;

    Slam slam = Slam(relativePath, initial_state, params);

    slam.cycle();
    cv::waitKey();
  }
  BOOST_AUTO_TEST_SUITE_END()
}
