//
// Created by Edoardo Ghini on 13/06/2018.
//

#include "Pendulum.hpp"

#define PI 3.14159265

namespace dyn_modeling {
    Pendulum::Pendulum(double t_delta_t, double t_length):
            m_state( std::vector<double>{ 50, 0}),
            m_clock( Clock(t_delta_t)),
            m_length( t_length),
            m_grav(9.81)
    {
    }

    std::vector<double> Pendulum::evolutionModel( std::vector<double> t_curr_state) {
        std::vector<double>  state_evolution = std::vector<double>{ 0,0 };
        state_evolution.at(0) = t_curr_state.at(1);
        state_evolution.at(1) =  - (m_grav/m_length)*sin(t_curr_state.at(0)*PI/180);
        return state_evolution;
        }

    void Pendulum::updateState() {
        std::vector<double> oldState = getState();
        std::vector<double> state_evolution = evolutionModel(oldState);


        Eigen::Vector2d  arr1;
        arr1 << oldState.at(0), oldState.at(1);
        Eigen::Vector2d  arr2;
        arr2 << state_evolution.at(0), state_evolution.at(1);
        Eigen::Vector2d arr3 = arr1 + arr2;

        std::vector<double> newState ={ arr3(0), arr3(1)} ;
        m_clock.thick();
        Pendulum::setState(newState);
    }


    void Pendulum::printState() {

        std::cout << "current time =" << m_clock.getCurrTime() << "\n";
        std::cout << "state 1=" << getState().at(0) << "\n";
        std::cout << "state 2=" << getState().at(1) << "\n";
    }

    void Pendulum::cycle(const int t_numCycles) {
        for (int i = 0; i < t_numCycles; i++) {
            Pendulum::updateState();
            Pendulum::printState();
        }
    }


}

















