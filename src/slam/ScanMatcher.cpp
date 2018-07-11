// Created by Dinies on 02/07/2018.

#include "ScanMatcher.hpp"

namespace dyn_modeling {
  ScanMatcher::ScanMatcher(){};

  std::vector<double> ScanMatcher::icpIterationRframe( const std::vector<double> &t_initialGuessState,const std::vector<scanPoint> &t_oldScanPoints_robot,const std::vector<scanPoint> &t_newScanPoints_robot){
    Eigen::Matrix<double, 3, 3> H;
    H.Zero();
    Eigen::Vector3d b;
    b.Zero();
    double chi = 0 ;
    Eigen::Matrix2d dR;
    dR << -sin(t_initialGuessState.at(2)), -cos(t_initialGuessState.at(2)),
      sin(t_initialGuessState.at(2)), cos(t_initialGuessState.at(2));
    Eigen::Vector2d p_r;
    Eigen::Vector2d z;
    Eigen::Isometry2d T;
    Eigen::Vector2d h_q;
    Eigen::Vector2d e;
    Eigen::Vector2d dhdTheta;
    Eigen::Matrix<double, 2, 3> J;

    for ( int i=0; i< t_newScanPoints_robot.size(); ++i){

      p_r = t_oldScanPoints_robot.at(i).coords;
      z = t_newScanPoints_robot.at(i).coords;
      T = MyMath::v2t(t_initialGuessState);
      h_q = T * p_r;

      e = h_q - z;
      dhdTheta = dR * p_r;

      J << 1 , 0 , dhdTheta(0),
        0, 1, dhdTheta(1);

      H = H + J.transpose() * J;
      b = b + J.transpose() * e;

      chi += e.transpose()* e;
    }
    Eigen::Vector3d dx = - H.inverse() * b;
    std::vector<double> delta_state = { dx(0),dx(1),dx(2)};
    return delta_state;
  };
}

