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
      Eigen::Vector3d initial_state(0,0,0);

      paramsSlam params = paramsSlam
        (
         0.001, // double icpEpsilon
         6, // int icpIterationsCap
         0.2, // double kernelThresholdScanMatching
         1.4, // double maxDistBetweenRangesLineMatcher
         1, // double maxAngularCoeffLineMatcher
         0.2, // double minLengthLinesLineMatcher
         15, // int maxCandidatesAssociation
         0.1, // double maxLengthDiffAssociation
         0.1, // double maxAbsoluteOrientationDiffThreshold
         0.1, // double maxNearLinesOrientationDiffThreshold
         1.3, // double nearLinesBonusScoreMultiplier
         2, // int numMiddleScanPoints
         0.05, // double borderRatio;
         0.02, //maxLinesLengthDiffLoopCloser;
         0.05, //maxLinesOrientDiffLoopCloser;
         4.5, //leafRangeKdtree;
         2.0, //maxDistancekdtree;
         0.2, //thresholdLoopRecognition;
         400, //everyNumIterTryLoopClosure
         0.2//ratioQuerySetLoopClosure
 
        );

      DatasetManager dM( relativePath);
      Robot robot(dM);
      ScanMatcher scanM(params.icpEpsilon);

      Drawer drawer(40);
      Map map( drawer );
      LineMatcher lineM(params.maxDistBetweenRangesLineMatcher,
          params.maxAngularCoeffLineMatcher,
          params.minLengthLinesLineMatcher);
      Graph graph( robot.getNumDataEntries(), robot.getNumRanges());
      LoopCloser<Graph> loopC(  graph,
          params.maxLinesLengthDiffLoopCloser,
          params.maxLinesOrientDiffLoopCloser,
          params.leafRangeKdtree,
          params.maxDistanceKdtree,
          params.thresholdLoopRecognition);

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
