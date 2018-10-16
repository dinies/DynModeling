// Created by Dinies on 16/10/2018.
#pragma once

#include <Eigen/Core>
#include "Slam.hpp"
#include "Map.hpp"
#include "Robot.hpp"
#include "ScanMatcher.hpp"
#include "LineMatcher.hpp"
#include "DataAssociator.hpp"
#include "Graph.hpp"
#include "LoopCloser.hpp"
#include "DatasetManager.hpp"
//TODO  every class now will have references to the injected dependencies,
//change the constructors of all of them

namespace dyn_modeling {

   class Initializator{

     private:

       std::string m_dataSetPath;
       Eigen::Vector3d m_initialRobotState;
       paramsSlam m_params;

  public:


    Initializator( const std::string &t_dataSetPath,
          const Eigen::Vector3d &t_initialRobotState,
          const paramsSlam &t_params);

    Slam& initalize();

    virtual ~Initializator() =default;
   };
}




