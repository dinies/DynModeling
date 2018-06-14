// Created by Edoardo Ghini on 13/06/2018.

#pragma once
//#include <Eigen/Core>
//#include <Eigen/Geometry>
//#include <Eigen/StdVector>
#include <unistd.h>
#include <iostream>
#include <vector>
#include "../utils/Clock.hpp"

namespace dyn_modeling {
    class Pendulum {

    private:
        std::vector<double> m_state;
        Clock m_clock;

    public:

        Pendulum();

        inline const std::vector<double> getState() const { return m_state; }

        inline void setState(std::vector<double> &t_newState) { m_state = t_newState; }

        void updateState();

        void printState();

        void cycle(int t_numCycles);

    };
}

