// Created by dinies on 6/07/2018.

#define BOOST_TEST_MODULE SlamTests

#include <iostream>
#include <boost/test/unit_test.hpp>
#include <Eigen/Core>

#include "../../src/slam/Map.hpp"
#include "../../include/structs.hpp"


namespace dyn_modeling{
  BOOST_AUTO_TEST_SUITE( MapClass)

  BOOST_AUTO_TEST_CASE(drawRobotTest) {

    Drawer drawer(15);
    Map m( drawer );
 
    state robotState;
    robotState.mu << 200.0,  200.0 , 1.0;
    const double radius = 3;
    std::vector< cv::Point2d> points = Map::computePointsRobot(robotState, radius);
    cv::Scalar blue = { 233, 0 , 0};
    m.drawRobot(points, blue);
    m.showImg();
    cv::waitKey();
  }

  BOOST_AUTO_TEST_SUITE_END()
}
