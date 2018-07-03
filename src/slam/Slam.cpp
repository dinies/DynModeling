// Created by Edoardo Ghini on 26/06/2018.

#include "Slam.hpp"
namespace dyn_modeling {


  Slam::Slam( const std::string &t_dataSet_AbsolPath, const std::vector<double> &t_initialRobotState):
    m_robot(  Robot( t_dataSet_AbsolPath, t_initialRobotState) ),
    m_scanMatcher( ScanMatcher()),
    m_map( Map())
  {}


  void Slam::cycle(){
    int num_dataEntries = 2 ;// m_robot.getNumDataEntries
    for (int i = 0; i < num_dataEntries; ++i) {
      std::vector<double> old_robotState = m_robot.getState();
      std::vector< scanPoint > scanPoints_robotFrame = m_robot.retrieveScanPoints(i);
      // remind the frame of the coords:  ICP calculations has to be maid wtr to robot instead of world
      std::vector<double> optimalTransf = m_scanMatcher.icpIteration(old_robotState, scanPoints_robotFrame);

      std::vector<double> new_robotState = m_robot.updateState( optimalTransf);

      //loop checker
      //m_map.drawRanges( scanPoints_robotFrame, new_robotState ); //TODO useful for debugging also
      // TODO consider that maybe the Map class should be a referenced by Robot class since only Robot would have the informations to draw the Map... to decide yet

    }
  }
}
