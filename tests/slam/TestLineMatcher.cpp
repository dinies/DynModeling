// Created by dinies on 18/08/2018.

#define BOOST_TEST_MODULE SlamTests

#include <iostream>
#include <boost/test/unit_test.hpp>
#include <Eigen/Core>

#include "../../src/slam/LineMatcher.hpp"
#include "../../src/slam/Robot.hpp"
#include "../../include/structs.hpp"


namespace dyn_modeling{
  BOOST_AUTO_TEST_SUITE( LineMatcherClass)

  BOOST_AUTO_TEST_CASE(dummyOneLine) {

    LineMatcher lM = LineMatcher(1, 2);

    line l1;
    l1.first_index = 0;
    l1.second_index = 1;
    std::vector<line> truth = { l1 } ;

    scanPoint sp1;
    scanPoint sp2;
    Eigen::Vector2d c1(0.1,0.1);
    Eigen::Vector2d c2(0.1,0.2);
    sp1.coords = c1;
    sp2.coords = c2;

    const std::vector<scanPoint> SPoints_robot ={ sp1, sp2};

    std::vector<line> result = lM.generateLines( SPoints_robot);
    line truth_line;
    line result_line;
    BOOST_CHECK_EQUAL( truth.size(), result.size());
    for ( int i = 0; i < truth.size() ; ++i){
       truth_line = truth.at(i);
       result_line = result.at(i);

      BOOST_CHECK_EQUAL( truth_line.first_index, result_line.first_index);
      BOOST_CHECK_EQUAL( truth_line.second_index, result_line.second_index);
    }
  }

  BOOST_AUTO_TEST_CASE(dummyTwoLines) {

    LineMatcher lM = LineMatcher(10, 0.1);

    line l1;
    l1.first_index = 0;
    l1.second_index = 1;
    line l2;
    l2.first_index = 1;
    l2.second_index = 2;

    std::vector<line> truth = { l1, l2};

    scanPoint sp1;
    scanPoint sp2;
    scanPoint sp3;
    Eigen::Vector2d c1(0.1,0.1);
    Eigen::Vector2d c2(0.1,0.2);
    Eigen::Vector2d c3(0.2,0.2);
    sp1.coords = c1;
    sp2.coords = c2;
    sp3.coords = c3;

    const std::vector<scanPoint> SPoints_robot ={ sp1, sp2, sp3};

    std::vector<line> result = lM.generateLines( SPoints_robot);
    line truth_line;
    line result_line;
    BOOST_CHECK_EQUAL( truth.size(), result.size());
    for ( int i = 0; i < truth.size() ; ++i){
       truth_line = truth.at(i);
       result_line = result.at(i);

      BOOST_CHECK_EQUAL( truth_line.first_index, result_line.first_index);
      BOOST_CHECK_EQUAL( truth_line.second_index, result_line.second_index);
    }

  }
  BOOST_AUTO_TEST_SUITE_END()
}
