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

  BOOST_AUTO_TEST_CASE(gridParameters) {

    std::string relativePath= "../files/datasets/realLaserScans.txt";
    Eigen::Vector3d initial_state(0.0, 0.0, 0.0);

    DatasetManager dM( relativePath);
    Robot r(dM);


    int num_data_entries = r.getNumDataEntries();
    double borderRatio = 0.05;
    std::vector< scanPoint > scanPoints =
      r.retrieveScanPointsRobotFrame(234,borderRatio);

    LineMatcher lM_1 = LineMatcher(0.2, 1, 0.1);
    LineMatcher lM_2 = LineMatcher(1.2, 1, 0.1);
    LineMatcher lM_3 = LineMatcher(1.2, 1, 0.15);
    LineMatcher lM_4 = LineMatcher(1.2, 1, 0.01);
    LineMatcher lM_5 = LineMatcher(1.3, 0.8, 0.2);
    LineMatcher lM_6 = LineMatcher(2, 1.2, 0.2);
    LineMatcher lM_7 = LineMatcher(1.4, 1, 0.2);
    LineMatcher lM_8 = LineMatcher(1.2, 1, 0.05);

    std::vector<std::vector< line>> linesMat
      {
       lM_1.generateLines( scanPoints),
       lM_2.generateLines( scanPoints),
       lM_3.generateLines( scanPoints),
       lM_4.generateLines( scanPoints),
       lM_5.generateLines( scanPoints),
       lM_6.generateLines( scanPoints),
       lM_7.generateLines( scanPoints),
       lM_8.generateLines( scanPoints)};

    RGBImage img_1;
    RGBImage img_2;
    RGBImage img_3;
    RGBImage img_4;
    RGBImage img_5;
    RGBImage img_6;
    RGBImage img_7;
    RGBImage img_8;
    RGBImage img_9;

    img_1.create( 500,600);
    img_1 = cv::Vec3b(227, 246, 253);
    cv::namedWindow("par1");
    cv::moveWindow("par1", 40, 40);

    img_2.create( 500,600);
    img_2 = cv::Vec3b(227, 246, 253);
    cv::namedWindow("par2");
    cv::moveWindow("par2", 640, 40);

    img_3.create( 500,600);
    img_3 = cv::Vec3b(227, 246, 253);
    cv::namedWindow("par3");
    cv::moveWindow("par3", 1240, 40);

    img_4.create( 500,600);
    img_4 = cv::Vec3b(227, 246, 253);
    cv::namedWindow("par4");
    cv::moveWindow("par4", 40, 570);

    img_5.create( 500,600);
    img_5 = cv::Vec3b(227, 246, 253);
    cv::namedWindow("par5");
    cv::moveWindow("par5", 640, 570);

    img_6.create( 500,600);
    img_6 = cv::Vec3b(227, 246, 253);
    cv::namedWindow("par6");
    cv::moveWindow("par6", 1240, 570);

    img_7.create( 500,600);
    img_7 = cv::Vec3b(227, 246, 253);
    cv::namedWindow("par7");
    cv::moveWindow("par7", 40, 1100);

    img_8.create( 500,600);
    img_8 = cv::Vec3b(227, 246, 253);
    cv::namedWindow("par8");
    cv::moveWindow("par8", 640, 1100);

    img_9.create( 500,600);
    img_9 = cv::Vec3b(227, 246, 253);
    cv::namedWindow("points");
    cv::moveWindow("points", 1240, 1100);


    cv::Scalar lightBlue= {210,139,38};
    cv::Scalar dark_red = {20,0,255};
    Drawer drawer(30);

    std::vector<RGBImage> images {img_1,
                                  img_2,
                                  img_3,
                                  img_4,
                                  img_5,
                                  img_6,
                                  img_7,
                                  img_8,
                                  img_9};

    int imgIndex = 0;

    for ( auto linesVec : linesMat){

      bool flagColor = false;

      for ( auto line : linesVec){
        scanPoint sP1 = scanPoints.at( line.first_index);
        scanPoint sP2 = scanPoints.at( line.second_index);
        cv::Point2d p1( sP1.coords(0), sP1.coords(1));
        cv::Point2d p2( sP2.coords(0), sP2.coords(1));

        if (flagColor){
          drawer.drawLine( images.at(imgIndex), p1, p2 , dark_red);
          flagColor = false;
        }
        else{
          drawer.drawLine( images.at(imgIndex), p1, p2 , lightBlue);
          flagColor = true;
        }
      }
      std::stringstream ss;
      ss << "par" << imgIndex + 1;
      cv::imshow(ss.str() , images.at(imgIndex));
      ++ imgIndex;
    }

    std::vector< cv::Point2d> points = Map::computePointsScans( scanPoints);
    for (auto p : points){
      drawer.drawPatch(img_9, p, lightBlue );
    }
    cv::imshow("points",img_9);

    cv::waitKey();
  }

  BOOST_AUTO_TEST_SUITE_END()
}
