// Created by Dinies on 16/10/2018.
//
#include "Initializator.hpp"
namespace dyn_modeling {


  Initializator::Initializator( const std::string &t_dataSetPath,
      const Eigen::Vector3d &t_initialRobotState,
      const paramsSlam &t_params):
    m_dataSetPath( t_dataSetPath),
    m_initialRobotState( t_initialRobotState),
    m_params( t_params)
  {};


  Slam& Initializator::initalize(){
    DatasetManager dM( m_dataSetPath);
    Robot( dM )
      //TODO

  Slam::Slam( const std::string &t_dataSet_AbsolPath,
              const Eigen::Vector3d &t_initialRobotState,
              const paramsSlam &t_params):

    m_params( t_params),
    m_robot(  Robot( t_dataSet_AbsolPath )),
    m_scanMatcher( ScanMatcher(t_params.icpEpsilon)),
    m_map( Map()),
    m_lineMatcher( LineMatcher(t_params.maxDistBetweenRangesLineMatcher,
                               t_params.maxAngularCoeffLineMatcher,
                               t_params.minLengthLinesLineMatcher)),
    m_initialRobotState( t_initialRobotState)
  {
    m_graph = Graph( m_robot.getNumDataEntries(), m_robot.getNumRanges());
    m_loopCloser = LoopCloser(  m_graph,
        m_params.maxLinesLengthDiffLoopCloser,
        m_params.maxLinesOrientDiffLoopCloser);
  }



  };
}

