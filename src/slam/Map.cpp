// Created by Dinies on 02/07/2018.

#include "Map.hpp"
namespace dyn_modeling {

  Map::Map():
    m_drawer( Drawer(10))
  {
    m_drawingImg.create( 900,1200 );
    m_drawingImg= cv::Vec3b(255, 255, 255);
    cv::namedWindow("Map");
    m_colors.white = {255,255,255};
    m_colors.green = {0,255,20};
    m_colors.dark_red = {20,0,255};
    m_spatialUnit = 0.3;
  };

  std::vector< cv::Point2d> Map::computePointsRobot( const std::vector<double> &t_robotState ){
    double robot_radius = m_spatialUnit;
    std::vector< cv::Point2d> pointVec;
    pointVec.reserve(6);
    double curr_angle(M_PI/6 + t_robotState.at(2));
    double angle_offset( M_PI/3);
    for (int i = 0; i < 6; ++i) {
      std::vector<double> curr_point = { robot_radius, 0 };
      MyMath::rotate2D( curr_point, curr_angle );
      cv::Point2d p( curr_point.at(0) + t_robotState.at(0), curr_point.at(1) + t_robotState.at(1));
      pointVec.push_back(p);
      curr_angle += angle_offset;
    }
    return pointVec;
  };

  std::vector< cv::Point2d> Map::computePointsScans( const std::vector<scanPoint> &t_scanPoints_worldFrame){
    std::vector< cv::Point2d> points;
    points.reserve( t_scanPoints_worldFrame.size());
    for( auto sP : t_scanPoints_worldFrame){
      cv::Point2d p( sP.coords(0), sP.coords(1));
      points.push_back(p);
    }
    return points;

  };

  void Map::drawRobot( const std::vector< cv::Point2d> &t_points, const cv::Scalar &t_color){
    m_drawer.drawHollowPoligon(m_drawingImg, t_points, t_color);
  };

  void Map::drawScans( const std::vector< cv::Point2d> &t_points, const cv::Scalar &t_color){
    for (auto p : t_points){
      m_drawer.drawPatch(m_drawingImg, p, t_color);
    }
 };


  void Map::drawScanPoints(const std::vector<scanPoint> &t_scanPoints_worldFrame,const std::vector<double> &t_robotState ,const int t_index){

    drawingData dD;
    dD.index = t_index;
    dD.robot_drawing.reserve(6);
    dD.scans_drawing.reserve(t_scanPoints_worldFrame.size());

    dD.robot_drawing = computePointsRobot( t_robotState);
    dD.scans_drawing = computePointsScans( t_scanPoints_worldFrame);

    drawRobot( dD.robot_drawing, m_colors.green);
    drawScans( dD.scans_drawing, m_colors.dark_red);

    m_drawingList.push_back(dD);
  };

  void Map::deleteScanPoints( const int t_index){

    int i( m_drawingList.size()-1);
    bool found(false);
    int eraseIndex(0);

    while ( i <= 0 || found){
      drawingData dD = m_drawingList.at(i);
      if ( dD.index == t_index){
        found = true;
        drawRobot( dD.robot_drawing, m_colors.white);
        drawScans( dD.scans_drawing, m_colors.white);
        eraseIndex = i;
      }
      --i;
    }

    if (found){
      m_drawingList.erase( m_drawingList.begin()+ eraseIndex);
    }
  };

}
