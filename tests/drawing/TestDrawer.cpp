// Created by Edoardo Ghini on 26/06/2018.

#define BOOST_TEST_MODULE SlamTests

#include <iostream>
#include <boost/test/unit_test.hpp>

#include "../../src/drawing/Drawer.hpp"

namespace dyn_modeling{

  BOOST_AUTO_TEST_CASE( simple_test2) {
    RGBImage dImg;
    dImg.create( 600, 900);
    dImg= cv::Vec3b(255, 255, 255);
    cv::namedWindow("Map");
    cv::Scalar dark_red = {20,0,255};

    cv::Point2d p1 ( 0 , 0);
    cv::Point2d p2 ( 1 , 0);
    cv::Point2d p3 ( 0 , 1);
    std::vector<cv::Point2d> points = { p1, p2, p3};

    Drawer::drawPoligon(dImg, points, dark_red);
    cv::imshow("Map",dImg);
    cv::waitKey();
  }
}
