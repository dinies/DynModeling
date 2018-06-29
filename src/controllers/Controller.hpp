// Created by Edoardo Ghini on 18/06/2018.
#pragma once

#include <iostream>
#include <Eigen/Core>
#include <vector>
namespace dyn_modeling {

    class Controller {
    private:
        std::vector<double> m_gains;
    public:
        Controller(std::vector<double> t_gains);

        std::vector<double> computeInput( double t_ref, Eigen::Vector2d t_state, double t_gravity_compens);
        double boxMinusAngleRad( double t_ref, double t_actual);
    };

}

