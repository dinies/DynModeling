// Created by Dinies on 02/07/2018.

#include "ScanMatcher.hpp"

namespace dyn_modeling {
  ScanMatcher::ScanMatcher(){};

  std::vector<double> ScanMatcher::icpIteration(const std::vector<double> &t_old_robotState, const std::vector<scanPoint> &t_scanPoints_robotFrame){
    return t_old_robotState;  //TODO dummy implementation
  };
}

