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

      //old params
      // paramsSlam params = paramsSlam( 6, 0.4, 3, 1, 0.1, 6, 0.1, 0.3, 3, 0.05);
      // paramsSlam params = paramsSlam( 6, 0.4, 0.6, 1, 0.1, 6, 0.1, 0.3, 3, 0.05);

      paramsSlam params = paramsSlam
        (
         0.1, // double icpEpsilon
         6, // int icpIterationsCap
         0.4, // double kernelThresholdScanMatching
         0.6, // double maxDistBetweenRangesLineMatcher
         1, // double maxAngularCoeffLineMatcher
         0.1, // double minLengthLinesLineMatcher
         6, // int maxCandidatesAssociation
         0.1, // double maxLengthDiffAssociation
         0.1, // double maxAbsoluteOrientationDiffThreshold
         0.1, // double maxNearLinesOrientationDiffThreshold
         1.3, // double nearLinesBonusScoreMultiplier
         3, // int numMiddleScanPoints
         0.05,// double borderRatio;
         0.1, //maxLinesLengthDiffLoopCloser;
         0.5 //maxLinesOrientDiffLoopCloser;
        );
      DatasetManager dM( relativePath);
      Robot robot(dM);
      ScanMatcher scanM(params.icpEpsilon);

      Drawer drawer(15);
      Map map( drawer );
      LineMatcher lineM(params.maxDistBetweenRangesLineMatcher,
          params.maxAngularCoeffLineMatcher,
          params.minLengthLinesLineMatcher);
      Graph graph( robot.getNumDataEntries(), robot.getNumRanges());
      LoopCloser loopC(  graph,
          params.maxLinesLengthDiffLoopCloser,
          params.maxLinesOrientDiffLoopCloser);

      Slam slam( 
          initial_state,
          params,
          robot,
          scanM,
          map,
          lineM,
          graph,
          loopC);




      slam.cycle();
      cv::waitKey();
    }
  BOOST_AUTO_TEST_SUITE_END()
}
