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


  class LineMatcher{

  private:
    double m_distanceBetweenRangesThreshold;
    double m_angularCoeffThreshold;
    double m_minLengthThreshold;
  public:
    LineMatcher( const double t_distanceBetweenRangesThreshold, const double t_angularCoeffThreshold, const double t_minLength);


    inline double getDistanceBetweenRangesThreshold() { return m_distanceBetweenRangesThreshold;};

    inline void setDistanceBetweenRangesThreshold( const double t_threshold) { m_distanceBetweenRangesThreshold= t_threshold; };

    inline double getAngularCoeffThreshold() { return m_angularCoeffThreshold;};

    inline void setAngularCoeffThreshold( const double t_threshold) { m_angularCoeffThreshold= t_threshold; };

    double computeLength(const std::vector<scanPoint> &t_scanPoints,  const int t_firstIndex, const int t_secondIndex );

    std::vector<line> generateLines(const std::vector<scanPoint> &t_scanPoints_worldFrame);



  };
}

