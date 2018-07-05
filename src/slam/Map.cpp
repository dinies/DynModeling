// Created by Dinies on 02/07/2018.

#include "Map.hpp"
namespace dyn_modeling {

  Map::Map(){
    m_drawingImg.create( 600, 900 );
    m_drawingImg= cv::Vec3b(255, 255, 255);
    cv::namedWindow("Map");
    m_colors.white = {255,255,255};
    m_colors.green = {0,255,20};
    m_colors.dark_red = {20,0,255};
    m_spatialUnit = 1;
  };

  std::vector< cv::Point2d> Map::computePointsRobot( const std::vector<double> &t_robotState ){
    double robot_radius = m_spatialUnit;
    std::vector< cv::Point2d> pointVec;
    pointVec.reserve(6);
    double curr_angle(M_PI/6);
    double angle_offset( M_PI/3);
    for (int i = 0; i < 6; ++i) {
      std::vector<double> curr_point = { robot_radius, 0 };
      MyMath::rotate2D( curr_point, curr_angle );
      cv::Point2d p( curr_point.at(0), curr_point.at(1));
      pointVec.push_back(p);
      curr_angle += angle_offset;
    }
    return pointVec;
  };

  std::vector< cv::Point2d> Map::computePointsScans( const std::vector<scanPoint> &t_scanPoints_worldFrame, const std::vector<double> &t_robotState){
    double scan_scale = m_spatialUnit/2;
    double edgeLength = scan_scale/sin(M_PI/3);
    std::vector< cv::Point2d> points;
    points.reserve( t_scanPoints_worldFrame.size()*3);

    for( auto sP : t_scanPoints_worldFrame){
      double angle_wtrWorld = std::atan2( sP.coords(1) - t_robotState.at(1), sP.coords(0) - t_robotState.at(0));
      cv::Point2d vertex1( sP.coords(0), sP.coords(1));
      cv::Point2d vertex2( sP.coords(0) + cos(angle_wtrWorld - M_PI/2)*edgeLength, sP.coords(1) + sin(angle_wtrWorld - M_PI/2)* edgeLength);
      cv::Point2d vertex3( sP.coords(0) - cos(angle_wtrWorld - M_PI/2)*edgeLength, sP.coords(1) - sin(angle_wtrWorld - M_PI/2)* edgeLength);
      points.push_back(vertex1);
      points.push_back(vertex2);
      points.push_back(vertex3);
    }

    return points;

  };

  void Map::drawRobot( const RGBImage &t_drawingImg, const std::vector< cv::Point2d> &t_points, const cv::Scalar &t_color){
    Drawer::drawPoligon(t_drawingImg, t_points, t_color);
  };

  void Map::drawScans( const RGBImage &t_drawingImg, const std::vector< cv::Point2d> &t_points, const cv::Scalar &t_color){
    int scan_num = t_points.size()/3;
    for (int i = 0; i < scan_num ; ++i) {
      int currIndex = i*3;
      std::vector< cv::Point2d> triangle = {
        t_points.at(currIndex),
        t_points.at(currIndex+1),
        t_points.at(currIndex+2)
      };
      Drawer::drawPoligon( t_drawingImg, triangle, t_color);
    }
  };


  void Map::drawScanPoints(const std::vector<scanPoint> &t_scanPoints_worldFrame,const std::vector<double> &t_robotState ,const int t_index){

    drawingData dD;
    dD.index = t_index;
    dD.robot_drawing.reserve(6);
    dD.scans_drawing.reserve(t_scanPoints_worldFrame.size() * 3);

    dD.robot_drawing = computePointsRobot( t_robotState);
    dD.scans_drawing = computePointsScans( t_scanPoints_worldFrame, t_robotState);

    Map::drawRobot( m_drawingImg, dD.robot_drawing, m_colors.green);
    Map::drawScans( m_drawingImg, dD.scans_drawing, m_colors.dark_red);

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
        Map::drawRobot( m_drawingImg, dD.robot_drawing, m_colors.white);
        Map::drawScans( m_drawingImg, dD.scans_drawing, m_colors.white);
        eraseIndex = i;
      }
      --i;
    }

    if (found){
      m_drawingList.erase( m_drawingList.begin()+ eraseIndex);
    }
  };

}
