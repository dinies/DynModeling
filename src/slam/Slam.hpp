// Created by Dinies on 26/06/2018.
#pragma once
#include <unistd.h>
#include <Eigen/Core>
// #include <boost/serialization/array_wrapper.hpp>
#include <vector>
// #include <math.h>
// #include "opencv2/opencv.hpp"

#include "../../include/gnuplot-iostream.h"
#include "Map.hpp"
#include "Robot.hpp"
#include "ScanMatcher.hpp"
#include "LineMatcher.hpp"
#include "DataAssociator.hpp"
#include "Graph.hpp"




namespace dyn_modeling {
  typedef struct resultMatchAssociations_tag{
    roundResult icpResult;
    std::vector<scanPoint> prevAssociationPoints;
    std::vector<scanPoint> currAssociationPoints;
  } resultMatchAssociations;

  class Slam {

  private:
    Robot m_robot;
    ScanMatcher m_scanMatcher;
    Map m_map;
    Graph m_graph;
    LineMatcher m_lineMatcher;


  public:
    Slam( const std::string &t_dataSet_AbsolPath, const Eigen::Vector3d &t_initialRobotState, const double t_maxDistBetweenRanges, const double t_maxAngularCoeff, const double t_minLength);
    void cycle();
    resultMatchAssociations matchAssociatedData(const node &t_prevNode,const edge &t_currEdge,const node &t_currNode, const int t_icpIterations_cap, const int numMidPoints);
    void preDrawingManagement( const int t_index);
    void postDrawingManagement( const int t_index);
    void plotStateEvolution(const double t_delta_t);

  };
}
