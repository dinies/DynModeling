// Created by Dinies on 02/07/2018.

#include "Robot.hpp"

namespace dyn_modeling {

  Robot::Robot( const std::string &t_dataSet_AbsolPath , const std::vector<double> &t_initial_state):
    m_datasetManager( DatasetManager( t_dataSet_AbsolPath))
  {
    m_state.q = t_initial_state;
    m_old_states.push_back(m_state);
  };

  std::vector<scanPoint> Robot::retrieveScanPoints( int t_index_datanode){
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
    }
    return scan_points;
  };

  std::vector<double> Robot::updateState(const std::vector<double> &optimalTransf){
    return  optimalTransf; //TODO dummy implementation, use v2t t2v given in defs.hpp library
  };
}


