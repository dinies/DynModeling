// Created by Dinies on 02/07/2018.

#pragma once
#include <unistd.h>
#include <vector>

#include "opencv2/opencv.hpp"



#include "../drawing/Drawer.hpp"
#include "../utils/MyMath.hpp"
#include "../../include/structs.hpp"

namespace dyn_modeling {

  typedef struct drawingData_tag{
    int index;
    state robot_state;
    std::vector<cv::Point2d> robot_drawing;
    std::vector<cv::Point2d> scans_drawing;
    std::vector<cv::Point2d> lineAssociations_drawing;
    bool someAssociations;
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
    RGBImage m_drawingLineAssociations;
    RGBImage m_drawingWorldMap;
    std::vector< drawingData> m_drawingList;
    colors m_colors;
    double m_spatialUnit;
    Drawer m_drawer;


  public:
    Map();

    static std::vector< cv::Point2d> computePointsRobot
    ( const state &t_robotState,
      const double t_robotRadius);

    static std::vector< cv::Point2d> computePointsScans
    ( const std::vector<scanPoint> &t_scanPoints_worldFrame);

    static std::vector< cv::Point2d> computePointsLineAssociations
    ( const std::vector<scanPoint> &t_prevAssociatedSP_worldFrame,
      const std::vector<scanPoint> &t_currAssociatedSP_worldFrame,
      const int t_numMiddlePoints);

    void drawRobot( const std::vector< cv::Point2d> &t_points,
                    const cv::Scalar &t_color);

    void drawScans( const std::vector< cv::Point2d> &t_points,
                    const cv::Scalar &t_color);

    void drawLineAssociations(  const std::vector< cv::Point2d> &t_points,
                                const cv::Scalar &t_prevColor,
                                const cv::Scalar &t_currColor);

    void drawWorldMap( const std::vector< cv::Point2d> &t_points,
                       const cv::Scalar &t_color);

    void drawImages
    (const std::vector<scanPoint> &t_scanPoints_worldFrame,
     const std::vector<scanPoint> &t_prevAssociatedSP_worldFrame,
     const std::vector<scanPoint> &t_currAssociatedSP_worldFrame,
     const int t_numMiddlePoints ,
     const state &t_robotState,
     const int t_index );

    void drawTrail(const int t_indexFrom, const int t_indexTo);

    void fadeScanPoints( const int t_index);

    void fadeRobot( const int t_index);

    void fadeWorldMap( const int t_index);

    void deleteScanPoints( const int t_index);

    void deleteRobot( const int t_index);

    void deleteLineAssociations( const int t_index);

    void showImg();

  };
}
