// Created by Edoardo Ghini on 26/06/2018.

#include "Slam.hpp"
namespace dyn_modeling {


  Slam::Slam( const std::string &t_dataSet_AbsolPath, const std::vector<double> &t_initialRobotState):
    m_robot(  Robot( t_dataSet_AbsolPath, t_initialRobotState) ),
    m_scanMatcher( ScanMatcher()),
    m_map( Map())
  {}


  void Slam::cycle(){
    int num_dataEntries = m_robot.getNumDataEntries();
    //here call a init func to preserve the space for datastructs in the objects for example in Map
    for (int i = 0; i < num_dataEntries; ++i) {

      std::vector<double> old_robotState;
      std::vector<scanPoint> oldSPoints_robotFrame;
      std::vector<scanPoint> oldSPoints_worldFrame;
      std::vector<scanPoint> newSPoints_robotFrame;
      std::vector<scanPoint> newSPoints_worldFrame;
      std::vector<double> new_robotState;

      if ( i == 0 ){
        new_robotState = m_robot.getState();
        newSPoints_robotFrame = m_robot.retrieveScanPointsRobotFrame(i);
        newSPoints_worldFrame = m_robot.changeCoordsRobotToWorld(newSPoints_robotFrame);

      }
      else{
        old_robotState = m_robot.getState();
        oldSPoints_robotFrame = m_robot.retrieveScanPointsRobotFrame(i - 1);

        oldSPoints_worldFrame = m_robot.changeCoordsRobotToWorld(oldSPoints_robotFrame);
        newSPoints_robotFrame = m_robot.retrieveScanPointsRobotFrame(i);
        newSPoints_worldFrame = m_robot.changeCoordsRobotToWorld(newSPoints_robotFrame);

        new_robotState = m_scanMatcher.icpIteration(old_robotState,oldSPoints_robotFrame,oldSPoints_worldFrame,newSPoints_worldFrame);

        m_robot.updateState( new_robotState);

      }
        //loop checker
        m_map.drawScanPoints( newSPoints_worldFrame , new_robotState , i);
        m_map.show();
        cv::waitKey(1);
    }
  }
}
