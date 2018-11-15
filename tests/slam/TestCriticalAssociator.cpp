// Created by dinies on 26/09/2018.

#define BOOST_TEST_MODULE SlamTests

#include <iostream>
#include <boost/test/unit_test.hpp>
#include "opencv2/opencv.hpp"
#include "../../src/slam/LineMatcher.hpp"
#include "../../src/slam/Robot.hpp"
#include "../../src/slam/Map.hpp"
#include "../../src/drawing/Drawer.hpp"
#include "../../include/structs.hpp"
#include "../../src/slam/DataAssociator.hpp"


namespace dyn_modeling{
  BOOST_AUTO_TEST_SUITE( DataAssociatorClass)


BOOST_AUTO_TEST_CASE(associateAcriticalIteration) {

    std::string relativePath= "../files/datasets/CriticalNoAssociations.txt";
    DatasetManager dM( relativePath);
    Robot r(dM);
 

    int num_data_entries = r.getNumDataEntries();
    double borderRatio = 0.05;
    std::vector< scanPoint > scanPoints_1 = r.retrieveScanPointsRobotFrame(0,borderRatio);
    std::vector< scanPoint > scanPoints_2 = r.retrieveScanPointsRobotFrame(1,borderRatio);

    LineMatcher lM = LineMatcher(0.6, 1, 0.1);
    // LineMatcher lM = LineMatcher(3, 1, 0.1);
    // LineMatcher lM = LineMatcher(100, 1, 0.1);


    std::vector<line> lines_1 = lM.generateLines( scanPoints_1);
    std::vector<line> lines_2 = lM.generateLines( scanPoints_2);

    RGBImage img_1;
    RGBImage img_2;
    RGBImage img_3;
    RGBImage img_4;
    RGBImage img_5;

    img_1.create( 600,800);
    img_1 = cv::Vec3b(227, 246, 253);
    cv::namedWindow("Lines1");
    cv::moveWindow("Lines1", 20, 20);

    img_2.create( 600,800);
    img_2 = cv::Vec3b(227, 246, 253);
    cv::namedWindow("Lines2");
    cv::moveWindow("Lines2", 840, 20);

    img_3.create( 600,800);
    img_3 = cv::Vec3b(227, 246, 253);
    cv::namedWindow("Points1");
    cv::moveWindow("Points1", 20, 740);

    img_4.create( 600,800);
    img_4 = cv::Vec3b(227, 246, 253);
    cv::namedWindow("Points2");
    cv::moveWindow("Points2", 840, 740);

    cv::Scalar lightBlue= {210,139,38};
    cv::Scalar dark_red = {20,0,255};
    Drawer drawer(30);

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

    const int max_candidates = 2;
    const double lengthDiffThreshold = 0.1;
    const double absoluteOrientationDiffThreshold = 0.1;
    const double nearLinesOrientationDiffThreshold = 0.3;
    const double nearLinesBonusScoreMultiplier= 1.3;


    DataAssociator dA = DataAssociator( max_candidates,
                                        lengthDiffThreshold,
                                        absoluteOrientationDiffThreshold,
                                        nearLinesOrientationDiffThreshold,
                                        nearLinesBonusScoreMultiplier,
                                        lines_1,
                                        scanPoints_1,
                                        lines_2,
                                        scanPoints_2);

    std::vector< dataAssociation> result = dA.associateLines();
    std::cout << "n of associations : " << result.size() << "\n";

     //std::vector<cv::Point2d> lineAssociations_drawing = computePointsLineAssociations( t_prevAssociatedSP_worldFrame, t_currAssociatedSP_worldFrame, t_numMiddlePoints);
    // drawLineAssociations( dD.lineAssociations_drawing, m_colors.dark_red, m_colors.lightBlue);


    cv::imshow("Lines1",img_1);
    cv::imshow("Lines2",img_2);
    cv::imshow("Points1",img_3);
    cv::imshow("Points2",img_4);

    cv::waitKey();
  }
  BOOST_AUTO_TEST_SUITE_END()
}
