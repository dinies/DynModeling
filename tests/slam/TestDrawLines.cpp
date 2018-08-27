// Created by dinies on 18/08/2018.

#define BOOST_TEST_MODULE SlamTests

#include <iostream>
#include <boost/test/unit_test.hpp>
#include <Eigen/Core>

#include "opencv2/opencv.hpp"
#include "../../src/slam/LineMatcher.hpp"
#include "../../src/slam/Robot.hpp"
#include "../../include/structs.hpp"
#include "../../src/drawing/Drawer.hpp"


namespace dyn_modeling{
  BOOST_AUTO_TEST_SUITE( LineMatcherClass)

  BOOST_AUTO_TEST_CASE(drawLinesDummyDataPoints) {
    scanPoint sp1;
    scanPoint sp2;
    scanPoint sp3;
    Eigen::Vector2d c1(0.1,0.1);
    Eigen::Vector2d c2(0.1,0.2);
    Eigen::Vector2d c3(0.2,0.2);
    sp1.coords = c1;
    sp2.coords = c2;
    sp3.coords = c3;

    const std::vector<scanPoint> scanPoints_robotFrame ={ sp1, sp2, sp3};

    LineMatcher lM = LineMatcher(0.1, 0.2);
    std::vector<line> lines = lM.generateLines( scanPoints_robotFrame);

    RGBImage img;
    img.create( 900,900);
    img = cv::Vec3b(227, 246, 253);
    cv::namedWindow("Lines");
    cv::moveWindow("Lines", 40, 40);
    cv::Scalar dark_red = {20,0,255};
    Drawer drawer(15);

    for ( auto line : lines){
      scanPoint sP1 = scanPoints_robotFrame.at( line.first_index);
      scanPoint sP2 = scanPoints_robotFrame.at( line.second_index);
      cv::Point2d p1( sP1.coords(0), sP1.coords(1));
      cv::Point2d p2( sP2.coords(0), sP2.coords(1));
      drawer.drawLine( img, p1, p2 , dark_red);
    }
    cv::waitKey();
  }

  // BOOST_AUTO_TEST_CASE(drawLinesRealDataPoints) {

  //   std::string absolutePath = "/home/dinies/gitrepos/DynModeling/files/datasets/exampleDataSetOneline.txt";
  //   std::vector<double> initial_state = { 0, 0, 0};
  //   Robot r = Robot(absolutePath, initial_state);

  //   int num_data_entries = r.getNumDataEntries();
  //   BOOST_CHECK_EQUAL( num_data_entries , 1);
  //   if (num_data_entries == 1){
  //     std::vector< scanPoint > scanPoints_robotFrame = r.retrieveScanPointsRobotFrame(0);
  //     LineMatcher lM = LineMatcher(0.1, 0.2);
  //     std::vector<line> lines = lM.generateLines( scanPoints_robotFrame);

  //     RGBImage img;
  //     img.create( 900,900);
  //     img = cv::Vec3b(227, 246, 253);
  //     cv::namedWindow("lines");
  //     cv::moveWindow("lines", 40, 40);
  //     cv::Scalar dark_red = {20,0,255};
  //     Drawer drawer(15);

  //     for ( auto line : lines){
  //       scanPoint sP1 = scanPoints_robotFrame.at( line.first_index);
  //       scanPoint sP2 = scanPoints_robotFrame.at( line.second_index);
  //       cv::Point2d p1( sP1.coords(0), sP1.coords(1));
  //       cv::Point2d p2( sP2.coords(0), sP2.coords(1));
  //       drawer.drawLine( img, p1, p2 , dark_red);
  //     }
  //     cv::waitKey();
  //   }
  // }
  BOOST_AUTO_TEST_SUITE_END()
}
