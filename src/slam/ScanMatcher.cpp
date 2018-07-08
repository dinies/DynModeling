// Created by Dinies on 02/07/2018.

#include "ScanMatcher.hpp"

namespace dyn_modeling {
  ScanMatcher::ScanMatcher(){};



  std::vector<double> ScanMatcher::icpIteration( const std::vector<double> &t_initialGuessState,const std::vector<scanPoint> &t_oldScanPoints_robot,const std::vector<scanPoint> &t_oldScanPoints_world, const std::vector<scanPoint> &t_newScanPoints_world){



    Eigen::Matrix<double, 3, 3> H;
    H.Zero();
    // H << 0,0,0,
    //   0,0,0,
    //   0,0,0;
    Eigen::Vector3d b;
    b.Zero();
    double chi = 0 ;
    // Eigen::Matrix2d R;
    // R << cos(state.at(2)), -sin(state.at(2)),
    //   sin(state.at(2)), cos(state.at(2));
    Eigen::Matrix2d dR;
    dR << -sin(t_initialGuessState.at(2)), -cos(t_initialGuessState.at(2)),
      sin(t_initialGuessState.at(2)), cos(t_initialGuessState.at(2));

    for ( int i=0; i< t_newScanPoints_world.size(); ++i){
      Eigen::Vector2d e = t_oldScanPoints_world.at(i).coords - t_newScanPoints_world.at(i).coords ;
      Eigen::Vector2d dhdTheta = dR * t_oldScanPoints_robot.at(i).coords;
      Eigen::Matrix<double, 2, 3> J;
      J << 1 , 0 , dhdTheta(0),
        0, 1, dhdTheta(1);
      H = H + J.transpose() * J;
      b = b + J.transpose() * e;

      chi += e.transpose()* e;
    }
    Eigen::Vector3d state( t_initialGuessState.at(0), t_initialGuessState.at(1), t_initialGuessState.at(2));
    Eigen::Vector3d dx = - H.inverse() * b;
    state = state + dx;
    std::vector<double> newStateVec = { state(0),state(1),state(2)};
    return newStateVec;
  };
}

