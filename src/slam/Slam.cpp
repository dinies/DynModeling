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
      std::vector<double> old_robotState = m_robot.getState();
      std::vector< scanPoint > scanPoints_robotFrame= m_robot.retrieveScanPointsRobotFrame(i);
      std::vector< scanPoint > scanPoints_worldFrame= m_robot.changeCoordsRobotToWorld( scanPoints_robotFrame );

      // remind the frame of the coords:  ICP calculations has to be maid wtr to robot instead of world (not sure)

      //std::vector<double> optimalTransf = m_scanMatcher.icpIteration(old_robotState, scanPoints_worldFrame);

      // std::vector<double> new_robotState = m_robot.updateState( optimalTransf);

      //loop checker
      // m_map.drawScanPoints( scanPoints_worldFrame, new_robotState , i);
      m_map.drawScanPoints( scanPoints_worldFrame, old_robotState , i);
    }
  }
}
