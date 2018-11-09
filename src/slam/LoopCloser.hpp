// Created by Dinies on 8/10/2018.

#pragma once
#include <unistd.h>
#include <vector>

#include "../../include/structs.hpp"
#include "GraphInterface.hpp"
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

  template <class T>
  class LoopCloser{

  private:
    T& m_graph;
    double m_maxLinesLengthDiff;
    double m_maxLinesOrientDiff;
    double m_leafRangeKdtree;
    double m_maxDistanceKdtree;
    double m_thresholdLoopRecognition;
 
  public:

    LoopCloser( T &t_graph,
                const double t_maxLinesLengthDiff,
                const double t_maxLinesOrientDiff,
                const double t_leafRangeKdtree,
                const double t_maxDistanceKdtree,
                const double t_thresholdLoopRecognition
                );

    std::vector<loopDrawingData> closeLoop( const int t_currIteration,
                    const int backRange,
                    const int querySetRange);

    static std::pair<int,int> findIndexesOptimization
    (const std::list< closure > &t_closures,
     const int t_currIteration);

    void sanitizeClosures( std::list< closure > &t_closures,
        std::vector<loopDrawingData> &t_loopDrawings);

    std::list<closure> findClosures (const int t_currIteration,
                                     const int t_backRange,
                                     const int t_querySetRange);

  };
}

#include "LoopCloser.tpp"
