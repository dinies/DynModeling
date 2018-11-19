// Created by Dinies on 02/07/2018.

#pragma once
#include <unistd.h>
#include <vector>
#include <limits>
// #include <string>
// #include <fstream>
// #include <iostream>
#include <Eigen/Dense>

#include "../../include/structs.hpp"
#include "../utils/MyMath.hpp"
#include "Robot.hpp"



namespace dyn_modeling {
  typedef struct iterResult_tag{
    double chi;
    Eigen::Vector3d delta_x;
    int outliers;
  } iterResult;


  typedef struct roundResult_tag{
    std::vector<double> chi;
    Eigen::Vector3d delta_x;
  } roundResult;



  class ScanMatcher{
  private:
    double m_kernelThreshold;
    double m_epsilon;
  public:
    ScanMatcher( const double t_epsilon);

    iterResult icpIterationRframe
    (const Eigen::Vector3d &t_initialGuessState,
     const std::vector<scanPoint> &t_oldScanPoints_robot,
     const std::vector<scanPoint> &t_newScanPoints_robot);

    roundResult icpRound(const int t_numIterations,
                         const Eigen::Vector3d &t_initialGuessState,
                         const std::vector<scanPoint> &t_oldScanPoints_robot,
                         const std::vector<scanPoint> &t_newScanPoints_robot);

    inline double getKernelThreshold() { return m_kernelThreshold;};
    inline void setKernelThreshold(const double t_threshold)
    { m_kernelThreshold = t_threshold; };
  };
}

