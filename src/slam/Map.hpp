// Created by Dinies on 02/07/2018.

#pragma once
#include <unistd.h>
#include <vector>

#include "opencv2/opencv.hpp"



#include "../drawing/Drawer.hpp"
#include "../utils/MyMath.hpp"

namespace dyn_modeling {

  typedef struct drawingData_tag{
    int index;
    std::vector<double> robot_state;
    std::vector<cv::Point2d> robot_drawing;
    std::vector<cv::Point2d> scans_drawing;
  }drawingData;

  typedef struct colors_tag{
    cv::Scalar green;
    cv::Scalar white;
    cv::Scalar dark_red;
    cv::Scalar milk;
    cv::Scalar lightBlue;
    cv::Scalar fadedLightBlue;
    cv::Scalar lightOrange;
    cv::Scalar darkBrown;
  }colors;




  class Map{
  private:
    RGBImage m_drawingMap;
    RGBImage m_drawingRobot;
    std::vector< drawingData> m_drawingList;
    colors m_colors;
    double m_spatialUnit;
    Drawer m_drawer;


  public:
    Map();

    std::vector< cv::Point2d> computePointsRobot( const std::vector<double> &t_robotState );

    std::vector< cv::Point2d> computePointsScans( const std::vector<scanPoint> &t_scanPoints_worldFrame);

    void drawRobot( const std::vector< cv::Point2d> &t_points, const cv::Scalar &t_color);

    void drawScans( const std::vector< cv::Point2d> &t_points, const cv::Scalar &t_color);

    void drawImages(const std::vector<scanPoint> &t_scanPoints_worldFrame,const std::vector<double> &t_robotState,const int t_index );

    void drawTrail(const int t_indexFrom, const int t_indexTo);

    void fadeScanPoints( const int t_index);

    void fadeRobot( const int t_index);

    void deleteScanPoints( const int t_index);

    void deleteRobot( const int t_index);

    void showImg();

  };
}
