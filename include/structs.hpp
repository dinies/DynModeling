#pragma once
#include <Eigen/Core>
#include "opencv2/opencv.hpp"

namespace dyn_modeling{

  typedef struct scanPoint_tag{
    Eigen::Vector2d coords;
  }scanPoint;

  typedef cv::Mat_< cv::Vec3b > RGBImage;

  typedef struct line_tag{
    int first_index;
    int second_index;
  } line;

  typedef struct dataAssociation_tag{
    int old_line_index;
    int new_line_index;
    double confidence_score;
  } dataAssociation;



}

