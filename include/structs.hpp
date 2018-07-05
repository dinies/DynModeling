#pragma once
#include <Eigen/Core>
#include "opencv2/opencv.hpp"

namespace dyn_modeling{

  typedef struct scanPoint_tag{
    Eigen::Vector2d coords;
  }scanPoint;

  typedef cv::Mat_< cv::Vec3b > RGBImage;
}

