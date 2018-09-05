// Created by Dinies on 02/07/2018.

#include "Robot.hpp"

namespace dyn_modeling {

  Robot::Robot( const std::string &t_dataSet_AbsolPath , const std::vector<double> &t_initial_state):
    m_datasetManager( DatasetManager( t_dataSet_AbsolPath))
  {
    m_state.mu = t_initial_state;
    m_state.sigma = Matrix3d::Zero();
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
    Eigen::Isometry2d transf = MyMath::v2t(m_state.mu);
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

  Eigen::Vector3d Robot::boxPlus(const Eigen::Vector3d &t_first,const Eigen::Vector3d &t_second){
    Eigen::Vector3d result(  t_first(0) + t_second(0),  t_first(1) + t_second(1), MyMath::boxPlusAngleRad(t_first(2), t_second(2)));
    return result;
  }

  Eigen::Vector3d Robot::boxMinus(const Eigen::Vector3d &t_first,const Eigen::Vector3d &t_second){
    Eigen::Vector3d result(  t_first(0) - t_second(0),  t_first(1) - t_second(1), MyMath::boxMinusAngleRad(t_first(2), t_second(2)));
    return result;
  }


  void Robot::updateState(const Eigen::Vector3d &t_deltaState){
    m_state.mu = Robot::boxPlus(m_state.mu, t_deltaState);
    Eigen::Isometry2d transf = MyMath::v2t(t_deltaState);
    Eigen::Matrix2d R = transf.linear();
    //TODO
    // m_state.sigma = 8;
  }
}





