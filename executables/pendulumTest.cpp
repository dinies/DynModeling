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
    const Eigen::Vector2d init_state (90, 0);
    const double t_delta_t = 0.01;
    const double t_length = 3;
    const double t_mass= 4;
    Pendulum p = Pendulum(init_state,t_delta_t, t_length,t_mass);
    p.cycle(numCycles);
//    Pendulum::state_type x(2);
//    x[0] = 5;
//    x[1] = 0;
//    double t_0 =0;
//    double t_f = t_0 + 1;
//    double initial_step_size = (t_f-t_0)/100;
//    std::size_t steps = boost::numeric::odeint::integrate( p ,x, t_0, t_f,initial_step_size );
    return 0;
}
