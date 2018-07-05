// Created by dinies on 03/07/2018.

#include "MyMath.hpp"

namespace dyn_modeling {


  Eigen::Isometry2d MyMath::v2t(const std::vector<double> &t_vec){

    Eigen::Isometry2d T;
    T.setIdentity();
    Eigen::Vector2d transl( t_vec.at(0), t_vec.at(1));
    T.translation()= transl;
    double c = cos(t_vec.at(2));
    double s = sin(t_vec.at(2));
    T.linear() << c, -s, s, c;
    return T;
  };


  std::vector<double> MyMath::t2v(const Eigen::Isometry2d& t_transf){

    Eigen::Vector3d v;
    v.head<2>()=t_transf.translation();
    v(2) = atan2(t_transf.linear()(1,0), t_transf.linear()(0,0));
    std::vector<double> vec = {v(0),v(1),v(2)};
    return vec;
  };


  void MyMath::rotate2D( std::vector<double> &t_point, const double t_angle_rad ){
    Eigen::Matrix2d R;
    R << cos(t_angle_rad ), -sin(t_angle_rad),
      sin(t_angle_rad), cos(t_angle_rad);
    Eigen::Vector2d p_old( t_point.at(0) , t_point.at(1));
    Eigen::Vector2d p_new = R* p_old;
    t_point.at(0) = p_new(0);
    t_point.at(1) = p_new(1);
  };
}
