// Created by Dinies on 26/06/2018.
#pragma once
#include <unistd.h>
#include <Eigen/Core>
// #include <boost/serialization/array_wrapper.hpp>
#include <vector>
// #include <math.h>
// #include "opencv2/opencv.hpp"
// #include "../../include/gnuplot-iostream.h"

#include "Map.hpp"
#include "Robot.hpp"
#include "ScanMatcher.hpp"


namespace dyn_modeling {
  class Slam {

  private:
    Robot m_robot;
    ScanMatcher m_scanMatcher;
    Map m_map;


  public:
    Slam( const std::string &t_dataSet_AbsolPath, const std::vector<double> &t_initialRobotState);
    void cycle();
    void drawingManagement( const int t_index);
  };
}
