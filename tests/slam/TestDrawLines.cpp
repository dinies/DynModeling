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
#include "../../src/slam/Map.hpp"


namespace dyn_modeling{
  BOOST_AUTO_TEST_SUITE( LineMatcherClass)

  BOOST_AUTO_TEST_CASE(drawLinesDummyDataPoints) {
    scanPoint sp1;
    scanPoint sp2;
    scanPoint sp3;
    Eigen::Vector2d c1(3,3);
    Eigen::Vector2d c2(3,6);
    Eigen::Vector2d c3(6,6);
    sp1.coords = c1;
    sp2.coords = c2;
    sp3.coords = c3;

    const std::vector<scanPoint> scanPoints_robotFrame ={ sp1, sp2, sp3};

    LineMatcher lM = LineMatcher(3, 0.2, 0.5);
    std::vector<line> lines = lM.generateLines( scanPoints_robotFrame);

    RGBImage img;
    img.create( 600,600);
    img = cv::Vec3b(227, 246, 253);
    cv::namedWindow("Lines1");
    cv::moveWindow("Lines1", 40, 40);
    cv::Scalar dark_red = {20,0,255};
    Drawer drawer(25);
    for ( auto line : lines){
      scanPoint sP1 = scanPoints_robotFrame.at( line.first_index);
      scanPoint sP2 = scanPoints_robotFrame.at( line.second_index);
      cv::Point2d p1( sP1.coords(0), sP1.coords(1));
      cv::Point2d p2( sP2.coords(0), sP2.coords(1));
      drawer.drawLine( img, p1, p2 , dark_red);
    }

    cv::imshow("Lines1",img);
    cv::waitKey(1);
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

    LineMatcher lM = LineMatcher(3, 0.2, 0.5);
    std::vector<line> lines = lM.generateLines( scanPoints_robotFrame);

    RGBImage img;
    img.create( 600,600);
    img = cv::Vec3b(227, 246, 253);
    cv::namedWindow("Lines2");
    cv::moveWindow("Lines2", 640, 40);
    cv::Scalar dark_red = {20,0,255};

    cv::Scalar lightBlue= {210,139,38};
    Drawer drawer(25);
    for ( auto line : lines){
      scanPoint sP1 = scanPoints_robotFrame.at( line.first_index);
      scanPoint sP2 = scanPoints_robotFrame.at( line.second_index);
      cv::Point2d p1( sP1.coords(0), sP1.coords(1));
      cv::Point2d p2( sP2.coords(0), sP2.coords(1));
      drawer.drawLine( img, p1, p2 , dark_red);
    }

    cv::imshow("Lines2",img);
    cv::waitKey(1);
  }


  BOOST_AUTO_TEST_CASE(drawLinesRealDataPoints) {

    std::string relativePath= "../files/datasets/exampleDataSetOneline.txt";
    Robot r = Robot(relativePath);

    int num_data_entries = r.getNumDataEntries();
    BOOST_CHECK_EQUAL( num_data_entries , 1);
    if (num_data_entries == 1){
      double borderRatio = 0.05;
      std::vector< scanPoint > scanPoints_robotFrame = r.retrieveScanPointsRobotFrame(0,borderRatio);
      LineMatcher lM = LineMatcher(0.2, 1, 0.1);
      std::vector<line> lines = lM.generateLines( scanPoints_robotFrame);

      std::cout << "num of lines : " << lines.size() << "\n" ;
      RGBImage img;
      img.create( 600,600);
      img = cv::Vec3b(227, 246, 253);
      cv::namedWindow("Lines3");
      cv::moveWindow("Lines3", 1240, 40);
      cv::Scalar dark_red = {20,0,255};
      cv::Scalar lightBlue= {210,139,38};

      Drawer drawer(30);

      bool flagColor = false;
      for ( auto line : lines){
        scanPoint sP1 = scanPoints_robotFrame.at( line.first_index);
        scanPoint sP2 = scanPoints_robotFrame.at( line.second_index);
        cv::Point2d p1( sP1.coords(0), sP1.coords(1));
        cv::Point2d p2( sP2.coords(0), sP2.coords(1));

        // std::cout << "line :" <<  sP1.coords(0) << ", "<< sP1.coords(1) << ";  "<< sP2.coords(0) << ", " << sP2.coords(1) << "\n" ;
        if (flagColor){
          drawer.drawLine( img, p1, p2 , dark_red);
          flagColor = false;
        }
        else{
          drawer.drawLine( img, p1, p2 , lightBlue);
          flagColor = true;
        }
      }

      cv::imshow("Lines3",img);
      cv::waitKey(1);
    }
  }


  BOOST_AUTO_TEST_CASE(drawLinesTwoSubsequentDataPoints) {

    std::string relativePath= "../files/datasets/realLaserScans.txt";
    Robot r = Robot(relativePath );

    int num_data_entries = r.getNumDataEntries();
    double borderRatio = 0.05;
    std::vector< scanPoint > scanPoints_1 = r.retrieveScanPointsRobotFrame(2343,borderRatio);
    std::vector< scanPoint > scanPoints_2 = r.retrieveScanPointsRobotFrame(2344,borderRatio);

    LineMatcher lM = LineMatcher(0.2, 1, 0.1);

    std::vector<line> lines_1 = lM.generateLines( scanPoints_1);
    std::vector<line> lines_2 = lM.generateLines( scanPoints_2);

    RGBImage img_1;
    RGBImage img_2;
    RGBImage img_3;
    RGBImage img_4;

    img_1.create( 700,700);
    img_1 = cv::Vec3b(227, 246, 253);
    cv::namedWindow("LinesA");
    cv::moveWindow("LinesA", 40, 670);

    img_2.create( 700,700);
    img_2 = cv::Vec3b(227, 246, 253);
    cv::namedWindow("LinesB");
    cv::moveWindow("LinesB", 1040, 970);

    img_3.create( 700,700);
    img_3 = cv::Vec3b(227, 246, 253);
    cv::namedWindow("PointsA");
    cv::moveWindow("PointsA", 740, 670);

    img_4.create( 700,700);
    img_4 = cv::Vec3b(227, 246, 253);
    cv::namedWindow("PointsB");
    cv::moveWindow("PointsB", 1740, 970);

    cv::Scalar lightBlue= {210,139,38};
    cv::Scalar dark_red = {20,0,255};
    Drawer drawer(10);

    bool flagColor = false;

    for ( auto line : lines_1){
      scanPoint sP1 = scanPoints_1.at( line.first_index);
      scanPoint sP2 = scanPoints_1.at( line.second_index);
      cv::Point2d p1( sP1.coords(0), sP1.coords(1));
      cv::Point2d p2( sP2.coords(0), sP2.coords(1));

      if (flagColor){
        drawer.drawLine( img_1, p1, p2 , dark_red);
        flagColor = false;
      }
      else{
        drawer.drawLine( img_1, p1, p2 , lightBlue);
        flagColor = true;
      }
    }
    for ( auto line : lines_2){
      scanPoint sP1 = scanPoints_2.at( line.first_index);
      scanPoint sP2 = scanPoints_2.at( line.second_index);
      cv::Point2d p1( sP1.coords(0), sP1.coords(1));
      cv::Point2d p2( sP2.coords(0), sP2.coords(1));

      if (flagColor){
        drawer.drawLine( img_2, p1, p2 , dark_red);
        flagColor = false;
      }
      else{
        drawer.drawLine( img_2, p1, p2 , lightBlue);
        flagColor = true;
      }
    }

    std::vector< cv::Point2d> points1 = Map::computePointsScans( scanPoints_1);
    for (auto p : points1){
      drawer.drawPatch(img_3, p, lightBlue );
    }

    std::vector< cv::Point2d> points2 = Map::computePointsScans( scanPoints_2);
    for (auto p : points2){
      drawer.drawPatch(img_4, p, lightBlue );
    }

    cv::imshow("LinesA",img_1);
    cv::imshow("LinesB",img_2);
    cv::imshow("PointsA",img_3);
    cv::imshow("PointsB",img_4);

    cv::waitKey();
  }

  BOOST_AUTO_TEST_SUITE_END()
}
