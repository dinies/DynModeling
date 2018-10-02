// Created by dinies on 30/08/2018.
#pragma once

#include <iostream>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace dyn_modeling {
  class Ellipse{

  public:
   static cv::RotatedRect getEllipse(double chisquare_val, cv::Point2f mean, cv::Mat covmat);

  };
}



