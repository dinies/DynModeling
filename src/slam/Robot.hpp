// Created by Dinies on 02/07/2018.

#pragma once
#include <unistd.h>
#include <Eigen/Core>
#include <boost/serialization/array_wrapper.hpp>
#include <boost/tuple/tuple.hpp>

#include "../../include/structs.hpp"
#include "../utils/MyMath.hpp"
#include "DatasetManager.hpp"
namespace dyn_modeling {

  class Robot{
  private:
    DatasetManager m_datasetManager;
    state m_state;

  public:
    Robot( const std::string &t_dataSet_AbsolPath , const Eigen::Vector3d &t_initial_state);

    inline state getState() { return m_state; };

    inline void setState(state &t_newState ) { m_state = t_newState; };

    std::vector<scanPoint> retrieveScanPointsRobotFrame( int t_index_datanode );

    std::vector<scanPoint> changeCoordsRobotToWorld( const std::vector<scanPoint> &t_scanPoints_robotFrame);

    void updateState(const Eigen::Vector3d &t_deltaState);

    inline int getNumDataEntries() { return m_datasetManager.getNumDataEntries(); }

    inline int getNumRanges() { return m_datasetManager.getNumRanges(); }


    static Eigen::Vector3d boxPlus(const Eigen::Vector3d &t_first,const Eigen::Vector3d &t_second);

    static Eigen::Vector3d boxMinus(const Eigen::Vector3d &t_first,const Eigen::Vector3d &t_second);

    static std::vector<scanPoint> computeMiddleScanPoints( const scanPoint &t_s1, const scanPoint &t_s2, const int t_numMidPoints);

  };
}
