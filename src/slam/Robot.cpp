// Created by Dinies on 02/07/2018.

#include "Robot.hpp"

namespace dyn_modeling {

  Robot::Robot( const std::string &t_dataSet_AbsolPath , const std::vector<double> &t_initial_state):
    m_datasetManager( DatasetManager( t_dataSet_AbsolPath))
  {
    m_state.q = t_initial_state;
    m_old_states.push_back(m_state);
  };



  std::vector<scanPoint> Robot::retrieveScanPointsRobotFrame( int t_index_datanode){
    std::vector<double> spanning_angles = m_datasetManager.getSpanningAngles();
    std::vector<double> data_ranges = m_datasetManager.getDataNodeRanges( t_index_datanode);
    std::vector<scanPoint> scan_points;
    scan_points.reserve(data_ranges.size());
    int index = 0;
    for ( auto angle : spanning_angles){
      scanPoint sP;
      Eigen::Matrix2d R;
      R << cos(angle ), -sin(angle),
        sin(angle), cos(angle);
      Eigen::Vector2d origin_range( data_ranges.at(index) ,0);
      Eigen::Vector2d coords = R* origin_range;
      sP.coords = coords;
      scan_points.push_back(sP);
      ++index;
    }
    return scan_points;
  };


  std::vector<scanPoint> Robot::changeCoordsRobotToWorld( const std::vector<scanPoint> &t_scanPoints_robotFrame){
    Eigen::Isometry2d transf = MyMath::v2t(m_state.q);
    std::vector<scanPoint> scanPvec_worldFrame;
    scanPvec_worldFrame.reserve( t_scanPoints_robotFrame.size());

    for ( auto sPoint : t_scanPoints_robotFrame){
      Eigen::Vector2d  sPvec_robot = sPoint.coords;
      scanPoint sPoint_world;
      sPoint_world.coords = transf * sPvec_robot;
      scanPvec_worldFrame.push_back( sPoint_world);
    }
    return scanPvec_worldFrame;
  };

  void Robot::updateState(const std::vector<double> &t_newState){
    m_old_states.push_back(m_state);
    // Eigen::Isometry2d transf = MyMath::v2t(t_transfVec);
    // Eigen::Vector2d curr_state( m_state.q.at(0), m_state.q.at(1));
    // Eigen::Vector2d new_state = transf * curr_state;
    // std::vector<double> new_state_vec(new_state(0), new_state(1));
    m_state.q = t_newState;
  };
}


