// Created by Dinies on 02/07/2018.

#include "ScanMatcher.hpp"

namespace dyn_modeling {
  ScanMatcher::ScanMatcher(){};

  roundResult ScanMatcher::icpRound(const int t_numIterations, const std::vector<double> &t_initialGuessState,const std::vector<scanPoint> &t_oldScanPoints_robot,const std::vector<scanPoint> &t_newScanPoints_robot){
    std::vector<double> chis;
    const double epsilon = 0.1;
    double chi = epsilon*2;
    int i = 0;
    iterResult curr_result;
    roundResult finalResult;
    finalResult.delta_x = {0 ,0 ,0};
    std::vector<double> curr_guess = t_initialGuessState;
    while (i < t_numIterations && chi>epsilon){
      curr_result = icpIterationRframe(curr_guess, t_oldScanPoints_robot, t_newScanPoints_robot);
      chi = curr_result.chi;
      chis.push_back(chi);
      curr_guess = Robot::boxPlus(curr_guess,curr_result.delta_x);
      finalResult.delta_x = MyMath::vecSum(finalResult.delta_x,curr_result.delta_x);
      ++i;
    }
    finalResult.chi = chis;
    return finalResult;
  };



  iterResult ScanMatcher::icpIterationRframe( const std::vector<double> &t_initialGuessState,const std::vector<scanPoint> &t_oldScanPoints_robot,const std::vector<scanPoint> &t_newScanPoints_robot){
    Eigen::Matrix<double, 3, 3> H;
    H.Zero();
    Eigen::Vector3d b;
    b.Zero();
    double chi = 0 ;
    Eigen::Matrix2d dR;
    dR << -sin(t_initialGuessState.at(2)), -cos(t_initialGuessState.at(2)),
      cos(t_initialGuessState.at(2)), -sin(t_initialGuessState.at(2));
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
      //TODO test v2t
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

    std::vector<double> delta_x= { dx(0),dx(1),dx(2)};
    iterResult res;
    res.delta_x = delta_x;
    res.chi = chi;
    return res;
  };
}

