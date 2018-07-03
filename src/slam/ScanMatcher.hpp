// Created by Dinies on 02/07/2018.

#pragma once
#include <unistd.h>
#include <vector>
// #include <string>
// #include <fstream>
// #include <iostream>

#include "../../include/structs.hpp"
namespace dyn_modeling {

  class ScanMatcher{
  public:
    ScanMatcher();

    std::vector<double> icpIteration( const std::vector<double> &t_old_robotState, const std::vector<scanPoint> &t_scanPoints_robotFrame);
  };
}

