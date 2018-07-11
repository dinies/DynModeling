// Created by Dinies on 02/07/2018.

#pragma once
#include <unistd.h>
#include <vector>
// #include <string>
// #include <fstream>
// #include <iostream>
#include <Eigen/Dense>

#include "../../include/structs.hpp"
#include "../utils/MyMath.hpp"


namespace dyn_modeling {


  class ScanMatcher{
  public:
    ScanMatcher();

    std::vector<double> icpIterationRframe( const std::vector<double> &t_initialGuessState,const std::vector<scanPoint> &t_oldScanPoints_robot,const std::vector<scanPoint> &t_newScanPoints_robot);

  };
}

