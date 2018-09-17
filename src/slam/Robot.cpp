// Created by Dinies on 02/07/2018.

#include "Robot.hpp"

namespace dyn_modeling {

  Robot::Robot( const std::string &t_dataSet_AbsolPath , const Eigen::Vector3d &t_initial_state):
    m_datasetManager( DatasetManager( t_dataSet_AbsolPath))
  {
    m_state.mu = t_initial_state;
    m_state.sigma = Eigen::Matrix3d::Zero();
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

  std::vector<scanPoint> Robot::computeMiddleScanPoints( const scanPoint &t_s1, const scanPoint &t_s2, const int t_numMidPoints){

    std::vector<scanPoint> middleScanPoints;
    middleScanPoints.reserve( t_numMidPoints);
    double dx = ( t_s2.coords(0) - t_s1.coords(0))/ (t_numMidPoints + 1);
    double dy = ( t_s2.coords(1) - t_s1.coords(1))/ (t_numMidPoints + 1);
    double x0{t_s1.coords(0) };
    double y0{t_s1.coords(1) };

    for ( int i = 1; i <= t_numMidPoints ; ++i){
      scanPoint s;
      x0 += dx;
      y0 += dy;
      s.coords << x0,y0;
      middleScanPoints.push_back(s);
    }
    return middleScanPoints;
  }

  Eigen::Vector3d Robot::boxPlus(const Eigen::Vector3d &t_x,const Eigen::Vector3d &t_delta_x){
    //TODO  rewrite see foundamental comment below
    Eigen::Isometry2d T_x = MyMath::v2t( t_x );
    Eigen::Isometry2d T_delta_x =MyMath::v2t( t_delta_x );
    return  MyMath::t2v( T_x * T_delta_x);
  }

  Eigen::Vector3d Robot::boxMinus(const Eigen::Vector3d &t_first,const Eigen::Vector3d &t_second){
    Eigen::Isometry2d T_first = MyMath::v2t( t_first );
    Eigen::Isometry2d T_second = MyMath::v2t( t_second );
    return  MyMath::t2v( T_first * T_second.inverse());
  }


  void Robot::updateState(const Eigen::Vector3d &t_deltaState){

    m_state.mu = Robot::boxPlus(m_state.mu, t_deltaState);
    // TODO foundamental the update is now in robot frame , now we need to transform the reference frame of the update so we need to work with homogeneous matrices so using v2t on the current state and on the delta x we obtain two matrices that have to be concatenated.

    // Eigen::Isometry2d transf = MyMath::v2t(t_deltaState);
    // Eigen::Matrix2d R = transf.linear();
    //TODO
    // m_state.sigma = 8;
  }
}





