// Created by Dinies on 02/07/2018.

#include "Map.hpp"
namespace dyn_modeling {

  Map::Map( Drawer &t_drawer):
    m_drawer( t_drawer)
  {
    m_drawingMap.create( 700,1200 );
    m_drawingMap= cv::Vec3b(227, 246, 253);
    cv::namedWindow("Map");
    cv::moveWindow("Map", 20, 20);
    m_drawingLineAssociations.create( 700,1200);
    m_drawingLineAssociations= cv::Vec3b(227, 246, 253);
    cv::namedWindow("LineAssociations");
    cv::moveWindow("LineAssociations", 1340, 20);

    m_drawingWorldMap.create( 700,900);
    m_drawingWorldMap = cv::Vec3b(227, 246, 253);
    cv::namedWindow("WorldMap");
    cv::moveWindow("WorldMap", 20, 840);

    m_drawingRobot.create( 300,300 );
    m_drawingRobot= cv::Vec3b(227, 246, 253);
    cv::namedWindow("Robot");
    cv::moveWindow("Robot", 940, 840);

    m_drawingLoops.create( 700,900);
    m_drawingWorldMap = cv::Vec3b(227, 246, 253);
    cv::namedWindow("Loops");
    cv::moveWindow("Loops", 1260, 840);

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

  std::vector< cv::Point2d> Map::computePointsRobot
    ( const state &t_robotState, const double t_robotRadius ){
      std::vector< cv::Point2d> pointVec;
      pointVec.reserve(6);
      double curr_angle(M_PI/6 + t_robotState.mu(2));
      double angle_offset( M_PI/3);
      for (int i = 0; i < 6; ++i) {
        Eigen::Vector2d currPoint(t_robotRadius, 0.0);
        MyMath::rotate2D( currPoint, curr_angle );
        cv::Point2d p( currPoint(0) , currPoint(1));
        pointVec.push_back(p);
        curr_angle += angle_offset;
      }
      return pointVec;
    };

  std::vector< cv::Point2d> Map::computePointsScans
    ( const std::vector<scanPoint> &t_scanPoints_worldFrame){
      std::vector< cv::Point2d> points;
      points.reserve( t_scanPoints_worldFrame.size());
      for( auto sP : t_scanPoints_worldFrame){
        cv::Point2d p( sP.coords(0), sP.coords(1));
        points.push_back(p);
      }
      return points;

    };

  std::vector< cv::Point2d> Map::computePointsLineAssociations
    ( const std::vector<scanPoint> &t_prevAssociatedSP_worldFrame,
      const std::vector<scanPoint> &t_currAssociatedSP_worldFrame,
      const int t_numMiddlePoints){

      std::vector< cv::Point2d> points;
      for (int i = 0; i< t_prevAssociatedSP_worldFrame.size();
          i+= t_numMiddlePoints ){
        scanPoint sP1 = t_prevAssociatedSP_worldFrame.at( i);
        scanPoint sP2 = t_prevAssociatedSP_worldFrame.at(i+ t_numMiddlePoints-1);
        scanPoint sP3 = t_currAssociatedSP_worldFrame.at( i);
        scanPoint sP4 = t_currAssociatedSP_worldFrame.at(i+ t_numMiddlePoints-1);

        cv::Point2d p1( sP1.coords(0) , sP1.coords(1));
        cv::Point2d p2( sP2.coords(0) , sP2.coords(1));
        cv::Point2d p3( sP3.coords(0) , sP3.coords(1));
        cv::Point2d p4( sP4.coords(0) , sP4.coords(1));

        points.push_back(p1);
        points.push_back(p2);
        points.push_back(p3);
        points.push_back(p4);
      }
      return points;
    }

  void Map::drawRobot( const std::vector< cv::Point2d> &t_points,
      const cv::Scalar &t_color){
    m_drawer.drawHollowPoligon(m_drawingRobot, t_points, t_color);
    cv::Point2d origin ( 0 , 0 );
    m_drawer.drawLine(m_drawingRobot, origin, t_points.at(0), t_color);
    m_drawer.drawLine(m_drawingRobot, origin, t_points.at(5), t_color);
  };

  void Map::drawScans( const std::vector< cv::Point2d> &t_points,
      const cv::Scalar &t_color){
    for (auto p : t_points){
      m_drawer.drawPatch(m_drawingMap, p, t_color);
    }
  };

  void Map::drawLineAssociations( const std::vector< cv::Point2d> &t_points,
      const cv::Scalar &t_prevColor,
      const cv::Scalar &t_currColor){
    for (int i = 0; i< t_points.size(); i+= 4 ){
      m_drawer.drawLine(m_drawingLineAssociations,
          t_points.at(i),
          t_points.at(i+1),
          t_prevColor);
      m_drawer.drawLine(m_drawingLineAssociations,
          t_points.at(i+2),
          t_points.at(i+3),
          t_currColor);
    }
  }

  void Map::drawWorldMap( const std::vector< cv::Point2d> &t_points,
      const cv::Scalar &t_color){
    for (int i = 0; i< t_points.size(); i+= 4 ){
      m_drawer.drawLine(m_drawingWorldMap,
          t_points.at(i),
          t_points.at(i+1),
          t_color);
    }
  }

  void Map::drawLoopClosures(
      const std::vector<loopDrawingData > &t_loops,
      const bool t_flagCurrentLoop){

    cv::Scalar colorCurrLine;
    cv::Scalar colorPrevLine;
    cv::Scalar colorCongiuntionLine;
    if ( t_flagCurrentLoop){

      colorCurrLine = m_colors.dark_red;
      colorPrevLine = m_colors.lightBlue;
      colorCongiuntionLine = m_colors.darkBrown;
    }
    else{
      colorCurrLine = m_colors.fadedLightBlue;
      colorPrevLine = m_colors.fadedLightBlue;;
      colorCongiuntionLine = m_colors.milk;
    }

    for (int i = 0; i< t_loops.size(); ++i ){
      loopDrawingData loop = t_loops.at(i);

      cv::Point2d currLine_firstEdge(
          loop.currLine_firstEdge(0),
          loop.currLine_firstEdge(1) );
      cv::Point2d currLine_secondEdge(
          loop.currLine_secondEdge(0),
          loop.currLine_secondEdge(1) );
      cv::Point2d currLine_middlePoint(
          loop.currLine_middlePoint(0),
          loop.currLine_middlePoint(1) );

      cv::Point2d prevLine_firstEdge(
          loop.prevLine_firstEdge(0),
          loop.prevLine_firstEdge(1) );
      cv::Point2d prevLine_secondEdge(
          loop.prevLine_secondEdge(0),
          loop.prevLine_secondEdge(1) );
      cv::Point2d prevLine_middlePoint(
          loop.prevLine_middlePoint(0),
          loop.prevLine_middlePoint(1) );

      m_drawer.drawLine(m_drawingLoops,
          currLine_firstEdge,
          currLine_secondEdge,
          colorCurrLine);
      m_drawer.drawLine(m_drawingLoops,
          prevLine_firstEdge,
          prevLine_secondEdge,
          colorPrevLine);
      m_drawer.drawLine(m_drawingLoops,
          currLine_middlePoint,
          prevLine_middlePoint,
          colorCongiuntionLine);

    }
  }

  void Map::drawImages
    (const std::vector<scanPoint> &t_scanPoints_worldFrame,
     const std::vector<scanPoint> &t_prevAssociatedSP_worldFrame,
     const std::vector<scanPoint> &t_currAssociatedSP_worldFrame,

     const int t_numMiddlePoints,
     const state &t_robotState,
     const int t_index ){

      drawingData dD;
      dD.index = t_index;
      dD.robot_drawing.reserve(6);
      dD.scans_drawing.reserve(t_scanPoints_worldFrame.size());
      dD.robot_state = t_robotState;
      dD.robot_drawing = computePointsRobot( t_robotState, m_spatialUnit*5);
      dD.scans_drawing = computePointsScans( t_scanPoints_worldFrame);
      drawRobot( dD.robot_drawing, m_colors.darkBrown);
      drawScans( dD.scans_drawing, m_colors.lightBlue);

      dD.someAssociations = t_prevAssociatedSP_worldFrame.size() != 0 &&
        t_currAssociatedSP_worldFrame.size() != 0;
      if ( dD.someAssociations){
        dD.lineAssociations_drawing.reserve(t_prevAssociatedSP_worldFrame.size());
        dD.lineAssociations_drawing =
          computePointsLineAssociations( t_prevAssociatedSP_worldFrame,
              t_currAssociatedSP_worldFrame,
              t_numMiddlePoints);
        drawLineAssociations( dD.lineAssociations_drawing,
            m_colors.dark_red,
            m_colors.lightBlue);
        drawWorldMap( dD.lineAssociations_drawing, m_colors.lightBlue);
      }
      m_drawingList.push_back(dD);

    };

  void Map::drawTrail(const int t_indexFrom, const int t_indexTo){

    if ( t_indexFrom <= t_indexTo && t_indexTo <= m_drawingList.size()-1){
      for (int i = t_indexFrom; i <= t_indexTo ; ++i) {
        drawingData dD = m_drawingList.at(i);
        cv::Point2d p( dD.robot_state.mu(0), dD.robot_state.mu(1));
        m_drawer.drawPatch(m_drawingMap, p, m_colors.darkBrown);
        m_drawer.drawPatch(m_drawingWorldMap, p, m_colors.dark_red);
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

  void Map::fadeWorldMap( const int t_index){
    if ( t_index <= m_drawingList.size()-1 ) {
      drawingData dD = m_drawingList.at(t_index);
      drawWorldMap( dD.lineAssociations_drawing, m_colors.lightOrange);
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

  void Map::deleteLineAssociations( const int t_index){
    if ( t_index <= m_drawingList.size()-1 ) {
      drawingData dD = m_drawingList.at(t_index);
      drawLineAssociations( dD.lineAssociations_drawing,
          m_colors.milk,
          m_colors.milk);
    }
  };



  void Map::showImg(){
    cv::imshow("Map",m_drawingMap);
    cv::imshow("Robot",m_drawingRobot);
    cv::imshow("LineAssociations",m_drawingLineAssociations);
    cv::imshow("WorldMap",m_drawingWorldMap);
    cv::imshow("Loops",m_drawingLoops);

  };
}
