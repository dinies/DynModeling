// Created by Dinies on 09/08/2018.

#pragma once
#include <unistd.h>
#include <vector>
// #include <limits>
// #include <string>
// #include <fstream>
// #include <iostream>
// #include <Eigen/Dense>

#include "../../include/structs.hpp"
#include "../utils/MyMath.hpp"
#include "Robot.hpp"



namespace dyn_modeling {

  typedef struct line_tag{
    int first_index;
    int second_index;
  } line;



  class LineMatcher{
  private:
    double m_kernelThreshold;
    double m_distanceBetweenSPointsThreshold;
    double m_angularCoeffThreshold;
  public:
    LineMatcher( const double t_kernelThreshold = 0.2, const double m_distanceBetweenSPointsThreshold = 0.5, const double m_angularCoeffThreshold = 0.1);

   inline double getKernelThreshold() { return m_kernelThreshold;};
   inline void setKernelThreshold( const double t_threshold) { m_kernelThreshold = t_threshold; };

    std::vector<line> generateLines(const std::vector<scanPoint> t_scanPoints_worldFrame);
  };
}

