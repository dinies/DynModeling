// Created by Dinies on 8/10/2018.

#pragma once
#include <unistd.h>
#include <vector>

// #include <limits>
// #include <string>
// #include <fstream>
// #include <iostream>
// #include <Eigen/Dense>

#include "../../include/structs.hpp"
#include "Graph.hpp"
#include "DataAssociator.hpp"
#include "Robot.hpp"
#include "../kdtree/kdTreeAltered.hpp"


namespace dyn_modeling {

  typedef struct closure_tag{
    trail newerTrail;
    trail olderTrail;
    double score;

    closure_tag( trail t1,
                 trail t2,
                 double score):
      newerTrail( t1),
      olderTrail( t2),
      score( score)
    {}
    closure_tag( trail t1,
                 trail t2):
      newerTrail( t1),
      olderTrail( t2),
      score( -1.0 )
    {}
    closure_tag():
      newerTrail(),
      olderTrail(),
      score( -1.0)
    {}

  } closure;

  class LoopCloser{

  private:
    Graph& m_graph;
    double m_maxDistCenters;
    double m_maxLinesLengthDiff;
    double m_maxLinesOrientDiff;

  public:
    LoopCloser(); 

    LoopCloser( Graph &t_graph,
                const double t_maxLinesLengthDiff,
                const double t_maxLinesOrientDiff);

    void closeLoop( const int t_currIteration,
                    const int backRange,
                    const int querySetRange);

    static std::pair<int,int> findIndexesOptimization
    (const std::list< closure > &t_closures,
     const int t_currIteration);

    void sanitizeClosures( std::list< closure > &t_closures);

    std::list<closure> findClosures (const int t_currIteration,
                                     const int t_backRange,
                                     const int t_querySetRange);

  };
}

