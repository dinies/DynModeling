//
// Created by Edoardo Ghini on 13/06/2018.
//
//
//#include "../include/defs.hpp"
//#include "../include/gnuplot-iostream.h"
//#include<iostream>
//#include <cmath>
//#include <boost/tuple/tuple.hpp>
//#include <opencv2/plot.hpp>

#include "src/pendulum/Pendulum.hpp"
using namespace dyn_modeling;

int main(int argc, char **argv) {
    const int numCycles{100};
    const double t_delta_t = 0.01;
    const double t_length = 2;
    Pendulum p = Pendulum(t_delta_t,t_length);
    p.cycle(numCycles);
    return 0;
}
