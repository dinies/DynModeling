// Created by Edoardo Ghini on 13/06/2018.
#include <iostream>
#include <stdlib.h>

#include "../src/drawing/Drawer.hpp"
#include "opencv2/opencv.hpp"

using namespace dyn_modeling;

int main(int argc, char **argv) {
  RGBImage dImg;
  dImg.create( 600, 900);
  dImg= cv::Vec3b(255, 255, 255);
  cv::namedWindow("Map");
  cv::Scalar dark_red = {20,0,255};

  cv::Point2d p1 ( 0 , 0);
  cv::Point2d p2 ( 1 , 0);
  cv::Point2d p3 ( 0 , 1);
  std::vector<cv::Point2d> points = { p1, p2, p3};

  std::abort();
  Drawer::drawPoligon(dImg, points, dark_red);
  cv::imshow("Map",dImg);
  cv::waitKey();
  return 0;
}


