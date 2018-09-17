// Created by dinies on 12/07/2018.

#define BOOST_TEST_MODULE SlamTests

#include <iostream>
#include <boost/test/unit_test.hpp>
#include <Eigen/Core>

#include "include/structs.hpp"
#include "src/slam/Robot.hpp"


namespace dyn_modeling{

  BOOST_AUTO_TEST_SUITE( Rob)
  BOOST_AUTO_TEST_CASE( retrieveScanPointsDummyDataset) {
    std::string relativePath= "../files/datasets/dummyDataSet.txt";
    Eigen::Vector3d initial_state (0.0, 0.0, 0.0);
    Robot r = Robot(relativePath, initial_state);

    BOOST_CHECK_EQUAL( r.getNumDataEntries(),1);
    if(r.getNumDataEntries()==1){
      std::vector< scanPoint > scanPoints_robotFrame = r.retrieveScanPointsRobotFrame(0);

      BOOST_CHECK_EQUAL( scanPoints_robotFrame.size(),5);
      if( scanPoints_robotFrame.size() == 5){
        std::vector<double> p1= {5, -8.66};
        std::vector<double> p2= {8.66, -5};
        std::vector<double> p3= {10, 0};
        std::vector<double> p4= {4.7631, 2.75};
        std::vector<double> p5= {0.5, 0.866};
        std::vector<std::vector<double>> truth_points = { p1, p2, p3, p4, p5};
        double threshold = 0.01;

        for( int i = 0; i < 5 ; ++i){
          BOOST_CHECK_SMALL( scanPoints_robotFrame.at(i).coords(0) - truth_points.at(i).at(0), threshold);
          BOOST_CHECK_SMALL( scanPoints_robotFrame.at(i).coords(1) - truth_points.at(i).at(1), threshold);
        }
      }
    }
 }

  BOOST_AUTO_TEST_CASE( retrieveScanPoints) {
    std::string relativePath= "../files/datasets/exampleDataSetOneline.txt";
    Eigen::Vector3d initial_state (0.0, 0.0, 0.0);
    Robot r = Robot(relativePath, initial_state);
    int num_data_entries = r.getNumDataEntries();
    BOOST_CHECK_EQUAL( num_data_entries , 1);
    if (num_data_entries == 1){
      std::vector< scanPoint > scanPoints_robotFrame = r.retrieveScanPointsRobotFrame(0);

      BOOST_CHECK_EQUAL( scanPoints_robotFrame.size() , 721);
      if (scanPoints_robotFrame.size() == 721){
        double angle = -((M_PI/2) - 0.5235984);
        Eigen::Matrix2d R;
        R << cos(angle ), -sin(angle),
          sin(angle), cos(angle);
        Eigen::Vector2d origin_range( 1.417 ,0);
        Eigen::Vector2d coords = R* origin_range;
        scanPoint sP = scanPoints_robotFrame.at(120);
        double threshold = 0.01;
        BOOST_CHECK_SMALL( sP.coords(0) - coords(0),threshold);
        BOOST_CHECK_SMALL( sP.coords(1) - coords(1),threshold);
      }
    }
  }

  BOOST_AUTO_TEST_CASE(updateState){
    std::string relativePath= "../files/datasets/exampleDataSetOneline.txt";
    Eigen::Vector3d initial_state (0.0, 0.0, 0.0);
    Robot r = Robot(relativePath, initial_state);
    Eigen::Vector3d delta_state( 4.0 , -2.0 , -M_PI/4);
    Eigen::Vector3d truth_state( 4.0 , -2.0 , -M_PI/4);
    r.updateState( delta_state);
    double threshold = 0.01;
    for ( int i = 0; i < 3; ++i){
      BOOST_CHECK_SMALL( r.getState().mu(i) - truth_state(i),threshold);
    }
  }

  BOOST_AUTO_TEST_CASE(computeMiddleScanPointsFirstQuadrant){
    scanPoint s1;
    scanPoint s2;
    s1.coords << 2, 2;
    s2.coords << 4, 4;

    std::vector<scanPoint> sVec = Robot::computeMiddleScanPoints( s1, s2, 1);

    BOOST_CHECK_EQUAL( sVec.size(), 1);
    if ( sVec.size() == 1){
      double threshold = 0.01;
      BOOST_CHECK_SMALL( sVec.at(0).coords(0) - 3,threshold);
      BOOST_CHECK_SMALL( sVec.at(0).coords(1) - 3,threshold);
    }
  }

  BOOST_AUTO_TEST_CASE(computeMiddleScanPointsFourthQuadrant){
    scanPoint s1;
    scanPoint s2;
    s1.coords << 1, -1;
    s2.coords << 9, -5;

    std::vector<scanPoint> sVec = Robot::computeMiddleScanPoints( s1, s2, 1);
    BOOST_CHECK_EQUAL( sVec.size(), 1);
    if ( sVec.size() == 1){
      double threshold = 0.01;
      BOOST_CHECK_SMALL( sVec.at(0).coords(0) - 5,threshold);
      BOOST_CHECK_SMALL( sVec.at(0).coords(1) + 3,threshold);
    }
  }

  BOOST_AUTO_TEST_CASE(computeMiddleScanPointsSecondQuadrant2points){
    scanPoint s1;
    scanPoint s2;
    s1.coords << -1, 1;
    s2.coords << -10, 7;
    std::vector<scanPoint> sVec = Robot::computeMiddleScanPoints( s1, s2, 2);

    BOOST_CHECK_EQUAL( sVec.size(), 2);
    if ( sVec.size() == 2){
      double threshold = 0.01;
      BOOST_CHECK_SMALL( sVec.at(0).coords(0) + 4,threshold);
      BOOST_CHECK_SMALL( sVec.at(0).coords(1) - 3,threshold);
      BOOST_CHECK_SMALL( sVec.at(1).coords(0) + 7,threshold);
      BOOST_CHECK_SMALL( sVec.at(1).coords(1) - 5,threshold);
    }
  }



  BOOST_AUTO_TEST_SUITE_END()
}

