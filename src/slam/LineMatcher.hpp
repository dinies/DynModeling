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
    double m_distanceBetweenSPointsThreshold;
    double m_angularCoeffThreshold;
  public:
    LineMatcher( double m_distanceBetweenSPointsThreshold , double m_angularCoeffThreshold );


    inline double getDistanceBetweenSPointsThreshold() { return m_distanceBetweenSPointsThreshold;};
    inline void setDistanceBetweenSPointsThreshold( const double t_threshold) { m_distanceBetweenSPointsThreshold= t_threshold; };

    inline double getAngularCoeffThreshold() { return m_angularCoeffThreshold;};
    inline void setAngularCoeffThreshold( const double t_threshold) { m_angularCoeffThreshold= t_threshold; };

    std::vector<line> generateLines(const std::vector<scanPoint> &t_scanPoints_worldFrame);
  };
}

