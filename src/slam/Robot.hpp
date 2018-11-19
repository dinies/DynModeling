// Created by Dinies on 02/07/2018.

#pragma once
#include <unistd.h>
#include <Eigen/Core>
// #include <boost/serialization/array_wrapper.hpp>
// #include <boost/tuple/tuple.hpp>
#include "DatasetManager.hpp"
#include "../utils/MyMath.hpp"
#include "../../include/structs.hpp"

namespace dyn_modeling {

  class Robot{
  private:
    DatasetManager &m_datasetManager;

  public:
    Robot( DatasetManager & t_datasetManager);


    std::vector<scanPoint> retrieveScanPointsRobotFrame
    ( const int t_index_datanode, const double borderRatio);

    bool checkScanPointInBorders( const double range,
                                  const double borderRatio);

    static std::vector<scanPoint> changeCoordsRobotToWorld
    ( const std::vector<scanPoint> &t_scanPoints_robotFrame,
      const state &t_currState);

    static state updateState( const state &t_currState,
                       const Eigen::Vector3d &t_deltaState);

    inline int getNumDataEntries()
    { return m_datasetManager.getNumDataEntries(); }

    inline int getNumRanges() { return m_datasetManager.getNumRanges(); }

    static Eigen::Vector3d elemWisePlus(const Eigen::Vector3d &t_x,
                                        const Eigen::Vector3d &t_delta_x);

    static Eigen::Vector3d elemWiseMinus(const Eigen::Vector3d &t_x,
                                         const Eigen::Vector3d &t_delta_x);

    static Eigen::Vector3d boxPlus(const Eigen::Vector3d &t_x,
                                   const Eigen::Vector3d &t_delta_x);

    static Eigen::Vector3d boxMinus(const Eigen::Vector3d &t_first,
                                    const Eigen::Vector3d &t_second);

    static std::vector<scanPoint> computeMiddleScanPoints
    ( const scanPoint &t_s1,
      const scanPoint &t_s2,
      const int t_numMidPoints);

  };
}
