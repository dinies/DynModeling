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
    double theta_0_rad= 0.3;
    double theta_0_dot_rad= 0.0;
    const Eigen::Vector2d init_state (theta_0_rad,theta_0_dot_rad);
    const double t_delta_t = 0.01;
    const double t_mass= 4;
    const double t_length = 7;
    const std::vector<double> t_gains{ 40, 5 };
    Pendulum p = Pendulum(init_state,t_delta_t, t_length,t_mass, t_gains);
    const double ref_rad= -0.9;
    p.cycle(numCycles, ref_rad, true);
    return 0;
}
