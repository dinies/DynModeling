// Created by Edoardo Ghini on 13/06/2018.

//#include "../include/defs.hpp"
//#include<iostream>


#include <boost/tuple/tuple.hpp>
//#include <Eigen/Core>
#include "../include/gnuplot-iostream.h"
#include "src/pendulum/Pendulum.hpp"

using namespace dyn_modeling;

int main(int argc, char **argv) {
    const int numCycles{10000};
    double theta_0_deg = 90;
    double theta_0_dot_deg = 0;
    const Eigen::Vector2d init_state (theta_0_deg*PI/180,theta_0_dot_deg*PI/180);
    const double t_delta_t = 0.01;
    const double t_length = 3;
    const double t_mass= 4;
    Pendulum p = Pendulum(init_state,t_delta_t, t_length,t_mass);
    p.cycle(numCycles);
    return 0;
}
