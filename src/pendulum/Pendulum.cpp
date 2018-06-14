//
// Created by Edoardo Ghini on 13/06/2018.
//

#include "Pendulum.hpp"

namespace dyn_modeling {
    Pendulum::Pendulum():m_state( std::vector<double>{ 0, 0}), m_clock( Clock(0.2))  {
    }


//  no pointers  only references
//    std::vector<double>*  Pendulum::incrementState(double t_inc) {
//        const std::vector<double>* oldStatePtr = Pendulum::getStatePtr();
//        const std::vector<double> oldState = *oldStatePtr;
//        std::vector<double> newState = { oldState.at(0) + t_inc,  oldState.at(1) + t_inc};
//        std::vector<double>* newState_ptr;
//        return newState_ptr;
//    }

    void Pendulum::updateState() {
        const double increment = 0.001;
        m_clock.thick();
        std::vector<double> oldState = getState();
        std::vector<double> newState = {oldState.at(0) + increment, oldState.at(1)-increment};
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

















