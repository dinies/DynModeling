// Created by dinies on 20/06/2018.

#pragma once
#include <unistd.h>
#include "opencv2/opencv.hpp"
#include "../../include/structs.hpp"

namespace dyn_modeling {

  typedef cv::Mat_< cv::Vec3b > RGBImage;
  class Drawer {

  public:

    static void drawPoligon(const RGBImage &t_drawing, const std::vector< cv::Point2d> &t_consecPoints, const cv::Scalar &t_color );

    static void drawLine(const RGBImage &t_drawing, const cv::Point2d &t_firstP, const cv::Point2d &t_secondP, const cv::Scalar &t_color);

    static cv::Point2d convertFrowWorldToImg(const RGBImage &t_drawing, const cv::Point2d &t_point);

  };
}
