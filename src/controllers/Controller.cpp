// Created by Edoardo Ghini on 18/06/2018.

#include "Controller.hpp"

namespace dyn_modeling {
    Controller::Controller(std::vector<double> t_gains): m_gains(t_gains){}

    std::vector<double> Controller::computeInput(double t_ref,Eigen::Vector2d t_state,double t_grav_comp) {

        double error = boxMinusAngleRad( t_ref, t_state(0));
        double K_p = m_gains.at(0);
        double K_d = m_gains.at(1);
        std::vector<double> vec{ K_p * error - K_d * t_state(1) + t_grav_comp, error};
        return vec;
    }

    double Controller::boxMinusAngleRad(double t_ref, double t_actual) {
           Eigen::Matrix2d rot_ref;
           Eigen::Matrix2d rot_actual;
           rot_ref << cos( t_ref ), -sin(t_ref),
                      sin(t_ref), cos(t_ref);
           rot_actual << cos( t_actual), -sin(t_actual),
                      sin(t_actual), cos(t_actual);
           Eigen::Matrix2d rot = rot_actual.transpose() * rot_ref;
           return atan2( rot(1,0), rot(0,0));

    }
}
