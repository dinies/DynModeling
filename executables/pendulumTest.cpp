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
    const double t_delta_t = 0.001;
    const double t_length = 1;
    Pendulum p = Pendulum(t_delta_t, t_length);
    p.cycle(numCycles);

    return 0;
}
