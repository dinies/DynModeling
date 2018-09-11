// Created by Dinies on 02/07/2018.

#include "Map.hpp"
namespace dyn_modeling {

  Map::Map():
    m_drawer( Drawer(15))
  {
    m_drawingMap.create( 900,1200 );
    m_drawingMap= cv::Vec3b(227, 246, 253);
    cv::namedWindow("Map");
    cv::moveWindow("Map", 20, 20);
    m_drawingRobot.create( 600,600 );
    m_drawingRobot= cv::Vec3b(227, 246, 253);
    cv::namedWindow("Robot");
    cv::moveWindow("Robot", 1240, 160);
    m_colors.white = {255,255,255};
    m_colors.green = {0,255,20};
    m_colors.dark_red = {20,0,255};
    m_colors.milk= {227,246,253};
    m_colors.lightBlue= {210,139,38};
    m_colors.fadedLightBlue= {255,207,130};
    m_colors.lightOrange = {0,164,216};
    m_colors.darkBrown = {1,83,109};
    m_spatialUnit = 0.4;
  };

  std::vector< cv::Point2d> Map::computePointsRobot( const state &t_robotState ){
    double robot_radius = m_spatialUnit*15;
    std::vector< cv::Point2d> pointVec;
    pointVec.reserve(6);
    double curr_angle(M_PI/6 + t_robotState.mu(2));
    double angle_offset( M_PI/3);
    for (int i = 0; i < 6; ++i) {
      Eigen::Vector2d currPoint(robot_radius, 0.0);
      MyMath::rotate2D( currPoint, curr_angle );
      cv::Point2d p( currPoint(0) , currPoint(1));
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
    m_drawer.drawHollowPoligon(m_drawingRobot, t_points, t_color);
    cv::Point2d origin ( 0 , 0 );
    m_drawer.drawLine(m_drawingRobot, origin, t_points.at(0), t_color);
    m_drawer.drawLine(m_drawingRobot, origin, t_points.at(5), t_color);
  };

  void Map::drawScans( const std::vector< cv::Point2d> &t_points, const cv::Scalar &t_color){
    for (auto p : t_points){
      m_drawer.drawPatch(m_drawingMap, p, t_color);
    }
  };

  void Map::drawImages(const std::vector<scanPoint> &t_scanPoints_worldFrame,const state &t_robotState ,const int t_index){
    drawingData dD;
    dD.index = t_index;
    dD.robot_drawing.reserve(6);
    dD.scans_drawing.reserve(t_scanPoints_worldFrame.size());
    dD.robot_state = t_robotState;
    dD.robot_drawing = computePointsRobot( t_robotState);
    dD.scans_drawing = computePointsScans( t_scanPoints_worldFrame);
    drawRobot( dD.robot_drawing, m_colors.darkBrown);
    drawScans( dD.scans_drawing, m_colors.lightBlue);

    m_drawingList.push_back(dD);
  };

  void Map::drawTrail(const int t_indexFrom, const int t_indexTo){

    if ( t_indexFrom <= t_indexTo && t_indexTo <= m_drawingList.size()-1){
      for (int i = t_indexFrom; i <= t_indexTo ; ++i) {
        drawingData dD = m_drawingList.at(i);
        cv::Point2d p( dD.robot_state.mu(0), dD.robot_state.mu(1));
        m_drawer.drawPatch(m_drawingMap, p, m_colors.darkBrown);
      }
    }
  };

  void Map::fadeScanPoints( const int t_index){
    if ( t_index <= m_drawingList.size()-1 ) {
      drawingData dD = m_drawingList.at(t_index);
      drawScans( dD.scans_drawing, m_colors.fadedLightBlue);
    }
  };

  void Map::fadeRobot( const int t_index){
    if ( t_index <= m_drawingList.size()-1 ) {
      drawingData dD = m_drawingList.at(t_index);
      drawRobot( dD.robot_drawing, m_colors.fadedLightBlue);
    }
  };

  void Map::deleteScanPoints( const int t_index){
    if ( t_index <= m_drawingList.size()-1 ) {
      drawingData dD = m_drawingList.at(t_index);
      drawScans( dD.scans_drawing, m_colors.milk);
    }
  };

  void Map::deleteRobot( const int t_index){
    if ( t_index <= m_drawingList.size()-1 ) {
      drawingData dD = m_drawingList.at(t_index);
      drawRobot( dD.robot_drawing, m_colors.milk);
    }
  };

  void Map::showImg(){
    cv::imshow("Map",m_drawingMap);
    cv::imshow("Robot",m_drawingRobot);
  };
}
