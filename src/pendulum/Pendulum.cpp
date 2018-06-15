//
// Created by Edoardo Ghini on 13/06/2018.
//

#include "Pendulum.hpp"

#define PI 3.14159265

namespace dyn_modeling {

    Pendulum::Pendulum(Eigen::Vector2d t_initial_state, double t_delta_t, double t_length):
            m_state( t_initial_state),
            m_clock( Clock(t_delta_t)),
            m_length( t_length),
            m_gravity(9.81)
            {}


    Eigen::Vector2d Pendulum::evolutionModel() {
        Eigen::Vector2d state_evolution;
        state_evolution(0) =  m_state(1);
        state_evolution(1) =  - (m_gravity/m_length)*sin(m_state(0)*PI/180);
        return state_evolution;
        }

    void Pendulum::updateState() {
        Eigen::Vector2d oldState = getState();
        Eigen::Vector2d state_evolution = evolutionModel();
        Eigen::Vector2d new_state = oldState + state_evolution;
        m_clock.thick();
        Pendulum::setState( new_state);
    }


    void Pendulum::printState() {
        std::cout << "current time =" << m_clock.getCurrTime() << '\n';
        std::cout << "state 1=" << getState()(0) << '\n';
        std::cout << "state 2=" << getState()(1) << '\n';
    }

//    void Pendulum::plotStateCycle(std::vector<std::vector<std::pair<double, double>>> &t_plotData) {
    void Pendulum::plotStateCycle(std::vector<std::vector<double>> &t_plotData) {

        std::vector< boost::tuple<double,double>> theta;
        std::vector< boost::tuple<double,double>> theta_dot;
        for ( auto vec : t_plotData){
            theta.push_back( boost::make_tuple( vec.at(0),vec.at(1)));
            theta_dot.push_back( boost::make_tuple( vec.at(0),vec.at(2)));
        }
        Gnuplot gp;
        gp << "set terminal qt 1 \n";
        gp << "plot";
        gp << gp.binFile1d(theta, "record") << "with lines title 'theta'" << "\n";
        gp << "set terminal qt 2 \n";
        gp << "plot";
        gp << gp.binFile1d(theta_dot, "record") << "with lines title 'theta dot'" << "\n";

    }

    void Pendulum::storePlotData(std::vector<std::vector<double>> &t_plotData) {
        double t = m_clock.getCurrTime();
        std::vector<double> curr_state { t, m_state(0), m_state(1)};
        t_plotData.push_back(curr_state);
    }

    void Pendulum::cycle(const int t_numCycles) {
        std::vector < std::vector<double>> plotting_data;
        for (int i = 0; i < t_numCycles; ++i) {
            Pendulum::updateState();
            Pendulum::storePlotData( plotting_data);
        }
        Pendulum::plotStateCycle(plotting_data);
    }

}


