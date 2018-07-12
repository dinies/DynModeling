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
    const int icpIterations_cap = 50;
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
      m_map.show();
      cv::waitKey(1);
    }
    m_robot.plotStateEvolution(0.01);
  }
}
