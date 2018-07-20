// Created by Edoardo Ghini on 26/06/2018.

#include "Slam.hpp"
namespace dyn_modeling {


  Slam::Slam( const std::string &t_dataSet_AbsolPath, const std::vector<double> &t_initialRobotState):
    m_robot(  Robot( t_dataSet_AbsolPath, t_initialRobotState) ),
    m_scanMatcher( ScanMatcher()),
    m_map( Map())
  {}


  void Slam::cycle(){
    const int num_ranges = m_robot.getNumRanges();
    const int num_dataEntries = m_robot.getNumDataEntries();
    const std::vector<double> initialGuessState = { 0, 0, 0};
    const int icpIterations_cap = 1;
    std::vector<double> old_robotState;
    old_robotState.reserve(3);
    std::vector<scanPoint> oldSPoints_robotFrame;
    oldSPoints_robotFrame.reserve(num_ranges);
    std::vector<scanPoint> newSPoints_robotFrame;
    newSPoints_robotFrame.reserve(num_ranges);
    std::vector<scanPoint> newDrawingPoints_worldFrame;
    newDrawingPoints_worldFrame.reserve(num_ranges);
    std::vector<double> new_robotState;
    new_robotState.reserve(3);
    roundResult icpRes;

    for (int i = 0; i < num_dataEntries; ++i) {

      if ( i == 0 ){
        new_robotState = m_robot.getState();
        newSPoints_robotFrame = m_robot.retrieveScanPointsRobotFrame(i);

      }
      else{
        oldSPoints_robotFrame = m_robot.retrieveScanPointsRobotFrame(i - 1);
        newSPoints_robotFrame = m_robot.retrieveScanPointsRobotFrame(i);
        icpRes = m_scanMatcher.icpRound(icpIterations_cap,initialGuessState,oldSPoints_robotFrame,newSPoints_robotFrame);

        m_robot.updateState( icpRes.delta_x);
        new_robotState = m_robot.getState();
      }
      //loop checker
      newDrawingPoints_worldFrame = m_robot.changeCoordsRobotToWorld(newSPoints_robotFrame);
      m_map.drawScanPoints( newDrawingPoints_worldFrame , new_robotState , i);
      drawingManagement(i);
    }
    m_robot.plotStateEvolution(0.01);
  }

  void  Slam::drawingManagement( const int t_index){
    const int i = t_index;
    // if (i>0){
      // m_map.deleteRobot(i-1);
    // }
    if (i%50==0) {
      m_map.drawTrail(0,i);
    }
    else{
      m_map.drawTrail(i,i);
    }

    if (i-20 > 0){
      m_map.fadeScanPoints(i-20);
    }
    if (i-200 > 0){
      m_map.deleteScanPoints(i-200);
    }

    m_map.show();
    cv::waitKey(1);
  }
}
