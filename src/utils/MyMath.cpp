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

  std::vector<double> MyMath::vecSum(const std::vector<double> &t_first,const std::vector<double> &t_second){
    std::vector<double> result;
    if ( t_first.size() == t_second.size() ){
      for (int i=0 ; i< t_first.size() ; ++i){
        result.push_back( t_first.at(i)+ t_second.at(i) );
      }
    }
    else{
      std::cout << "vecSum received vectors of different size, sum is not possible. \n";
      result = {0};
    }
    return result;
  };

  std::vector<double> MyMath::vecMultEleWise(const std::vector<double> &t_first,const std::vector<double> &t_second){
    std::vector<double> result;
    if ( t_first.size() == t_second.size() ){
      for (int i=0 ; i< t_first.size() ; ++i){
        result.push_back( t_first.at(i) * t_second.at(i) );
      }
    }
    else{
      std::cout << "vecSum received vectors of different size, sum is not possible. \n";
      result = {0};
    }
    return result;
  };


  double MyMath::boxMinusAngleRad(const double t_ref,const double t_actual) {
    Eigen::Matrix2d rot_ref;
    Eigen::Matrix2d rot_actual;
    rot_ref << cos( t_ref ), -sin(t_ref),
      sin(t_ref), cos(t_ref);
    rot_actual << cos( t_actual), -sin(t_actual),
      sin(t_actual), cos(t_actual);
    Eigen::Matrix2d rot = rot_actual.transpose() * rot_ref;
    return atan2( rot(1,0), rot(0,0));
  };

  double MyMath::boxPlusAngleRad(const double t_ref,const double t_actual) {
    Eigen::Matrix2d rot_ref;
    Eigen::Matrix2d rot_actual;
    rot_ref << cos( t_ref ), -sin(t_ref),
      sin(t_ref), cos(t_ref);
    rot_actual << cos( t_actual), -sin(t_actual),
      sin(t_actual), cos(t_actual);
    Eigen::Matrix2d rot = rot_actual * rot_ref;
    return atan2( rot(1,0), rot(0,0));
  };


}
