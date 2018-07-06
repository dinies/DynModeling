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
    std::vector<cv::Point2d> robot_drawing;
    std::vector<cv::Point2d> scans_drawing;
  }drawingData;

  typedef struct colors_tag{
    cv::Scalar green;
    cv::Scalar white;
    cv::Scalar dark_red;
  }colors;




  class Map{
  private:
    RGBImage m_drawingImg;
    std::vector< drawingData> m_drawingList;
    colors m_colors;
    int m_spatialUnit;


  public:
    Map();

    std::vector< cv::Point2d> computePointsRobot( const std::vector<double> &t_robotState );

    std::vector< cv::Point2d> computePointsScans( const std::vector<scanPoint> &t_scanPoints_worldFrame);

    void drawRobot( const std::vector< cv::Point2d> &t_points, const cv::Scalar &t_color);

    void drawScans( const std::vector< cv::Point2d> &t_points, const cv::Scalar &t_color);

    void drawScanPoints(const std::vector<scanPoint> &t_scanPoints_worldFrame,const std::vector<double> &t_robotState,const int t_index );

    void deleteScanPoints( const int t_index);

    inline void show(){ cv::imshow("Map",m_drawingImg); };

  };
}
