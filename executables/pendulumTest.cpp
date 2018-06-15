// Created by Edoardo Ghini on 13/06/2018.

//#include "../include/defs.hpp"
//#include<iostream>


#include <boost/tuple/tuple.hpp>
//#include <Eigen/Core>
#include "../include/gnuplot-iostream.h"
#include "src/pendulum/Pendulum.hpp"

using namespace dyn_modeling;

int main(int argc, char **argv) {
    const int numCycles{1000};
    const Eigen::Vector2d init_state (1, 0);
    const double t_delta_t = 0.01;
    const double t_length = 1;
    Pendulum p = Pendulum(init_state,t_delta_t, t_length);
    p.cycle(numCycles);
    return 0;
}
