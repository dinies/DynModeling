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
    double icpEpsilon;
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
    paramsSlam_tag(double t_1, int t_2, double t_3,double t_4,double t_5,
                   double t_6,int t_7,double t_8,double t_9,
                   double t_10,double t_11,int t_12,double t_13):
      icpEpsilon( t_1),
      icpIterationsCap( t_2),
      kernelThresholdScanMatching(t_3),
      maxDistBetweenRangesLineMatcher(t_4),
      maxAngularCoeffLineMatcher(t_5),
      minLengthLinesLineMatcher(t_6),
      maxCandidatesAssociation(t_7),
      maxLengthDiffAssociation(t_8),
      maxAbsoluteOrientationDiffThreshold(t_9),
      maxNearLinesOrientationDiffThreshold(t_10),
      nearLinesBonusScoreMultiplier(t_11),
      numMiddleScanPoints(t_12),
      borderRatio( t_13)
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

    Slam( const std::string &t_dataSet_AbsolPath,
          const Eigen::Vector3d &t_initialRobotState,
          const paramsSlam &t_params);

    void cycle();

    resultMatchAssociations matchAssociatedData(const node &t_prevNode,
                                                const edge &t_currEdge,
                                                const node &t_currNode,
                                                const int t_icpIterations_cap,
                                                const int numMidPoints);

    void preDrawingManagement( const int t_index);
    void postDrawingManagement( const int t_index);
    void plotStateEvolution(const double t_delta_t);

  };
}
