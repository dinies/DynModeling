// Created by Dinies on 02/07/2018.

#pragma once
#include <unistd.h>
#include <Eigen/Core>
#include <boost/serialization/array_wrapper.hpp>
#include <boost/tuple/tuple.hpp>

#include "../../include/gnuplot-iostream.h"
#include "../../include/structs.hpp"
#include "../utils/MyMath.hpp"
#include "DatasetManager.hpp"
namespace dyn_modeling {
  typedef struct state_tag{
    std::vector<double> q;
  } state;

  class Robot{
  private:
    DatasetManager m_datasetManager;
    state m_state;
    std::vector<state> m_old_states;

  public:
    Robot( const std::string &t_dataSet_AbsolPath , const std::vector<double> &t_initial_state);

    inline std::vector<double> getState() { return m_state.q; };

    std::vector<scanPoint> retrieveScanPointsRobotFrame( int t_index_datanode );

    std::vector<scanPoint> changeCoordsRobotToWorld( const std::vector<scanPoint> &t_scanPoints_robotFrame);

    void updateState(const std::vector<double> &t_deltaState);

    inline int getNumDataEntries() { return m_datasetManager.getNumDataEntries(); }

    inline int getNumRanges() { return m_datasetManager.getNumRanges(); }


    void plotStateEvolution(const double t_delta_t);

    static std::vector<double> boxPlus(const std::vector<double> t_first,const std::vector<double> t_second);
    static std::vector<double> boxMinus(const std::vector<double> t_first,const std::vector<double> t_second);

  };
}
