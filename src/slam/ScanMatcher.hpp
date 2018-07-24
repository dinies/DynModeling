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
    std::vector<double> delta_x;
    int outliers;
  } iterResult;


  typedef struct roundResult_tag{
    std::vector<double> chi;
    std::vector<double> delta_x;
  } roundResult;



  class ScanMatcher{
  private:
    double m_kernelThreshold;
  public:
    ScanMatcher();

    iterResult icpIterationRframe( const std::vector<double> &t_initialGuessState,const std::vector<scanPoint> &t_oldScanPoints_robot,const std::vector<scanPoint> &t_newScanPoints_robot);

    roundResult icpRound(const int t_numIterations, const std::vector<double> &t_initialGuessState,const std::vector<scanPoint> &t_oldScanPoints_robot,const std::vector<scanPoint> &t_newScanPoints_robot);

    inline double getKernelThreshold() { return m_kernelThreshold;};
    inline void setKernelThreshold( const double t_threshold) { m_kernelThreshold = t_threshold; };
  };
}

