#pragma once
#include <Eigen/Core>
#include "opencv2/opencv.hpp"

namespace dyn_modeling{

  typedef struct scanPoint_tag{
    Eigen::Vector2d coords;

    scanPoint_tag( double t_x, double t_y):
      coords(t_x, t_y)
    {}
    scanPoint_tag(Eigen::Vector2d t_coords):
      coords(t_coords )
    {}
    scanPoint_tag():
      coords(0,0)
    {}
  }scanPoint;

  typedef cv::Mat_< cv::Vec3b > RGBImage;

  typedef struct line_tag{
    int first_index;
    int second_index;

    line_tag( int t_1, int t_2):
      first_index( t_1),
      second_index( t_2)
    {}
    line_tag():
      first_index(-1),
      second_index(-1)
    {}
  } line;

  typedef struct dataAssociation_tag{
    int old_line_index;
    int new_line_index;
    double confidence_score;
    dataAssociation_tag( int t_1, int t_2, double t_3):
      old_line_index( t_1),
      new_line_index( t_2),
      confidence_score( t_3)
    {}
    dataAssociation_tag():
      old_line_index(-1),
      new_line_index(-1),
      confidence_score(-1)
    {}
  } dataAssociation;

  typedef struct state_tag{
    Eigen::Vector3d mu;
    Eigen::Matrix3d sigma;

    state_tag( Eigen::Vector3d t_1,
               Eigen::Matrix3d t_2):
      mu( t_1),
      sigma( t_2)
    {}
    state_tag( Eigen::Vector3d t):
      mu( t)
    { sigma =  Eigen::Matrix3d::Zero(); }
    state_tag( double t_x, double t_y, double t_theta ):
      mu( t_x, t_y, t_theta)
    { sigma =  Eigen::Matrix3d::Zero(); }
    state_tag(){
      mu = Eigen::Vector3d::Zero();
      sigma =  Eigen::Matrix3d::Zero();
    }
  } state;

  typedef struct trail_tag{
    Eigen::VectorXd lineCenterCoords;
    int nodeIndex;
    int lineIndex;
    int length;

    trail_tag( Eigen::VectorXd t_1,
               int t_2,
               int t_3,
               int t_4):
      lineCenterCoords( t_1),
      nodeIndex( t_2),
      lineIndex( t_3),
      length( t_4)
    {}
    trail_tag():
      nodeIndex(-1),
      lineIndex(-1),
      length(-1)
    {
      lineCenterCoords = Eigen::Vector2d::Zero();
    }
  } trail;

}





