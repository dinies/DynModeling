// Created by dinies on 03/07/2018.

//Collection of static functions and useful types to perform geometric computations
#pragma once

#include <unistd.h>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace dyn_modeling {
  class MyMath{

  public:
    static Eigen::Isometry2d v2t(const std::vector<double> &t_vec);

    static std::vector<double> t2v(const Eigen::Isometry2d& t_transf);

    static void rotate2D( std::vector<double> &t_point, const double t_angle_rad );

    static std::vector<double> vecSum(const std::vector<double> &t_first,const std::vector<double> &t_second);

    static double boxMinusAngleRad(double t_ref, double t_actual);
  };
}
