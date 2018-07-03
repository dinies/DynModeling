// Created by Dinies on 02/07/2018.

#pragma once
#include <unistd.h>
#include <Eigen/Core>
// #include <vector>
// #include <string>
// #include <fstream>
// #include <iostream>

#include "../../include/structs.hpp"
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

    std::vector<scanPoint> retrieveScanPoints( int t_index_datanode );

    std::vector<double> updateState( const std::vector<double> &optimalTransf);


  };
}
