// Created by Edoardo Ghini on 13/06/2018.

#pragma once
#include <Eigen/Core>
//#include <Eigen/Geometry>
//#include <Eigen/StdVector>
#include <unistd.h>
#include <boost/serialization/array_wrapper.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/numeric/odeint.hpp>
#include <iostream>
#include <vector>
#include <math.h>
#include "../utils/Clock.hpp"
#include "../../include/gnuplot-iostream.h"

namespace dyn_modeling {
    class Pendulum {


    private:
        Eigen::Vector2d m_state;
        Clock m_clock;
        double m_length;
        double m_mass;
        double m_gravity;

        Eigen::Vector2d evolutionModel();


        void updateState();

        void printState();

        void plotStateCycle(std::vector< std::vector<double>>& t_plotData);

        void storePlotData(std::vector< std::vector<double>>& t_plotData, double t_curr_input);

        double computeEnergy();




    public:

        typedef std::vector< double > state_type;

//        Pendulum( double t_delta_t, double t_length );
        Pendulum(const Eigen::Vector2d t_initial_state, double t_delta_t, double t_length, double t_mass );

        inline const Eigen::Vector2d getState() const { return m_state; }

        inline void setState(Eigen::Vector2d &t_newState) { m_state = t_newState; }

        void cycle(int t_numCycles);

        void operator() ( const state_type &x , state_type &dxdt , double /* t */ );

    };
}

