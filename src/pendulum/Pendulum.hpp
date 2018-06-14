// Created by Edoardo Ghini on 13/06/2018.

#pragma once
#include <Eigen/Core>
//#include <Eigen/Geometry>
//#include <Eigen/StdVector>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <math.h>
#include "../utils/Clock.hpp"

namespace dyn_modeling {
    class Pendulum {

    private:
        std::vector<double> m_state;
        Clock m_clock;
        double m_length;
        double m_grav;

    public:

        Pendulum( double t_delta_t, double t_length );

        inline const std::vector<double> getState() const { return m_state; }

        inline void setState(std::vector<double> &t_newState) { m_state = t_newState; }

        std::vector<double> evolutionModel(std::vector<double> t_curr_state);

        void updateState();

        void printState();

        void cycle(int t_numCycles);

    };
}

