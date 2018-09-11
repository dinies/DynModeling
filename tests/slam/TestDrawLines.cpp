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
    Eigen::Vector2d c1(10,10);
    Eigen::Vector2d c2(10,20);
    Eigen::Vector2d c3(20,20);
    sp1.coords = c1;
    sp2.coords = c2;
    sp3.coords = c3;

    const std::vector<scanPoint> scanPoints_robotFrame ={ sp1, sp2, sp3};

    LineMatcher lM = LineMatcher(100, 0.2);
    std::vector<line> lines = lM.generateLines( scanPoints_robotFrame);

    RGBImage img;
    img.create( 900,900);
    img = cv::Vec3b(227, 246, 253);
    cv::namedWindow("Lines");
    cv::moveWindow("Lines", 40, 40);
    cv::Scalar dark_red = {20,0,255};
    Drawer drawer(15);
    cv::imshow("Lines",img);
    for ( auto line : lines){
      scanPoint sP1 = scanPoints_robotFrame.at( line.first_index);
      scanPoint sP2 = scanPoints_robotFrame.at( line.second_index);
      cv::Point2d p1( sP1.coords(0), sP1.coords(1));
      cv::Point2d p2( sP2.coords(0), sP2.coords(1));
      drawer.drawLine( img, p1, p2 , dark_red);
    }

    cv::imshow("Lines",img);
    cv::waitKey(500);
  }

  BOOST_AUTO_TEST_CASE(drawLinesThreeSquareAnglesPoints) {
    scanPoint sp1;
    scanPoint sp2;
    scanPoint sp3;
    scanPoint sp4;
    scanPoint sp5;
    Eigen::Vector2d c1(5,7);
    Eigen::Vector2d c2(6,7);
    Eigen::Vector2d c3(6,6);
    Eigen::Vector2d c4(7,6);
    Eigen::Vector2d c5(7,5);
    sp1.coords = c1;
    sp2.coords = c2;
    sp3.coords = c3;
    sp4.coords = c4;
    sp5.coords = c5;

    const std::vector<scanPoint> scanPoints_robotFrame ={ sp1, sp2, sp3, sp4, sp5};

    LineMatcher lM = LineMatcher(100, 0.2);
    std::vector<line> lines = lM.generateLines( scanPoints_robotFrame);

    RGBImage img;
    img.create( 900,900);
    img = cv::Vec3b(227, 246, 253);
    cv::namedWindow("Lines");
    cv::moveWindow("Lines", 40, 40);
    cv::Scalar dark_red = {20,0,255};
    Drawer drawer(25);
    cv::imshow("Lines",img);
    for ( auto line : lines){
      scanPoint sP1 = scanPoints_robotFrame.at( line.first_index);
      scanPoint sP2 = scanPoints_robotFrame.at( line.second_index);
      cv::Point2d p1( sP1.coords(0), sP1.coords(1));
      cv::Point2d p2( sP2.coords(0), sP2.coords(1));
      drawer.drawLine( img, p1, p2 , dark_red);
    }

    cv::imshow("Lines",img);
    cv::waitKey(500);
  }


  BOOST_AUTO_TEST_CASE(drawLinesRealDataPoints) {

    std::string relativePath= "../files/datasets/exampleDataSetOneline.txt";
    Eigen::Vector3d initial_state(0.0, 0.0, 0.0);
    Robot r = Robot(relativePath, initial_state);

    int num_data_entries = r.getNumDataEntries();
    BOOST_CHECK_EQUAL( num_data_entries , 1);
    if (num_data_entries == 1){
      std::vector< scanPoint > scanPoints_robotFrame = r.retrieveScanPointsRobotFrame(0);
      LineMatcher lM = LineMatcher(5, 1);
      std::vector<line> lines = lM.generateLines( scanPoints_robotFrame);

      std::cout << "num of lines : " << lines.size() << "\n" ;
      RGBImage img;
      img.create( 900,900);
      img = cv::Vec3b(227, 246, 253);
      cv::namedWindow("Lines");
      cv::moveWindow("Lines", 40, 40);
      cv::Scalar dark_red = {20,0,255};
      Drawer drawer(30);

      cv::imshow("Lines",img);
      for ( auto line : lines){
        scanPoint sP1 = scanPoints_robotFrame.at( line.first_index);
        scanPoint sP2 = scanPoints_robotFrame.at( line.second_index);
        cv::Point2d p1( sP1.coords(0), sP1.coords(1));
        cv::Point2d p2( sP2.coords(0), sP2.coords(1));

        // std::cout << "line :" <<  sP1.coords(0) << ", "<< sP1.coords(1) << ";  "<< sP2.coords(0) << ", " << sP2.coords(1) << "\n" ;
        drawer.drawLine( img, p1, p2 , dark_red);
      }

      cv::imshow("Lines",img);
      cv::waitKey(1000);
    }
  }


  BOOST_AUTO_TEST_CASE(drawLinesTwoSubsequentDataPoints) {

    std::string relativePath= "../files/datasets/realLaserScans.txt";
    Eigen::Vector3d initial_state(0.0, 0.0, 0.0);
    Robot r = Robot(relativePath, initial_state);

    int num_data_entries = r.getNumDataEntries();
    std::vector< scanPoint > scanPoints_1 = r.retrieveScanPointsRobotFrame(1100);
    std::vector< scanPoint > scanPoints_2 = r.retrieveScanPointsRobotFrame(1300);

    LineMatcher lM = LineMatcher(5, 1);
    std::vector<line> lines_1 = lM.generateLines( scanPoints_1);
    std::vector<line> lines_2 = lM.generateLines( scanPoints_2);

    RGBImage img_1;
    RGBImage img_2;

    img_1.create( 900,900);
    img_1 = cv::Vec3b(227, 246, 253);
    cv::namedWindow("Lines 1");
    cv::moveWindow("Lines 1", 40, 40);

    img_2.create( 900,900);
    img_2 = cv::Vec3b(227, 246, 253);
    cv::namedWindow("Lines 2");
    cv::moveWindow("Lines 2", 960, 40);

    cv::Scalar dark_red = {20,0,255};
    cv::imshow("Lines 1",img_1);
    cv::imshow("Lines 2",img_2);
    Drawer drawer(30);

    for ( auto line : lines_1){
      scanPoint sP1 = scanPoints_1.at( line.first_index);
      scanPoint sP2 = scanPoints_1.at( line.second_index);
      cv::Point2d p1( sP1.coords(0), sP1.coords(1));
      cv::Point2d p2( sP2.coords(0), sP2.coords(1));

      drawer.drawLine( img_1, p1, p2 , dark_red);
    }
    for ( auto line : lines_2){
      scanPoint sP1 = scanPoints_2.at( line.first_index);
      scanPoint sP2 = scanPoints_2.at( line.second_index);
      cv::Point2d p1( sP1.coords(0), sP1.coords(1));
      cv::Point2d p2( sP2.coords(0), sP2.coords(1));

      drawer.drawLine( img_2, p1, p2 , dark_red);
    }

    cv::imshow("Lines 1",img_1);
    cv::imshow("Lines 2",img_2);

    cv::waitKey();
  }

  BOOST_AUTO_TEST_SUITE_END()
}
