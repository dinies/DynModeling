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

  typedef struct paramsSlam_tag{
    int icpIterationsCap;
    double kernelThresholdScanMatching;
    double maxDistBetweenRangesLineMatcher;
    double maxAngularCoeffLineMatcher;
    double minLengthLinesLineMatcher;
    int maxCandidatesAssociation;
    double maxLengthDiffAssociation;
    double maxAbsoluteOrientationDiffThreshold;
    double maxNearLinesOrientationDiffThreshold;
    double nearLinesBonusScoreMultiplier;
    int numMiddleScanPoints;
    double borderRatio;
    paramsSlam_tag(int t_1,double t_2,double t_3,double t_4,
                   double t_5,int t_6,double t_7,double t_8,
                   double t_9,double t_10,int t_11,double t_12):
      icpIterationsCap( t_1),
      kernelThresholdScanMatching(t_2),
      maxDistBetweenRangesLineMatcher(t_3),
      maxAngularCoeffLineMatcher(t_4),
      minLengthLinesLineMatcher(t_5),
      maxCandidatesAssociation(t_6),
      maxLengthDiffAssociation(t_7),
      maxAbsoluteOrientationDiffThreshold(t_8),
      maxNearLinesOrientationDiffThreshold(t_9),
      nearLinesBonusScoreMultiplier(t_10),
      numMiddleScanPoints(t_11),
      borderRatio( t_12)
    {}
  } paramsSlam;

  typedef struct resultMatchAssociations_tag{
    roundResult icpResult;
    std::vector<scanPoint> prevAssociationPoints;
    std::vector<scanPoint> currAssociationPoints;
  } resultMatchAssociations;

  class Slam {

  private:
    paramsSlam m_params;
    Robot m_robot;
    ScanMatcher m_scanMatcher;
    Map m_map;
    Graph m_graph;
    LineMatcher m_lineMatcher;


  public:

    Slam( const std::string &t_dataSet_AbsolPath, const Eigen::Vector3d &t_initialRobotState, const paramsSlam &t_params);
    void cycle();
    resultMatchAssociations matchAssociatedData(const node &t_prevNode,const edge &t_currEdge,const node &t_currNode, const int t_icpIterations_cap, const int numMidPoints);
    void preDrawingManagement( const int t_index);
    void postDrawingManagement( const int t_index);
    void plotStateEvolution(const double t_delta_t);

  };
}
