//
// Created by Edoardo Ghini on 13/06/2018.
//

#include "Pendulum.hpp"

#define PI 3.14159265

namespace dyn_modeling {
    Pendulum::Pendulum(double t_delta_t, double t_length):
            m_state( std::vector<double>{ 5, 0}),
            m_clock( Clock(t_delta_t)),
            m_length( t_length),
            m_gravity(9.81)
    {
    }

    std::vector<double> Pendulum::evolutionModel( std::vector<double> t_curr_state) {
        std::vector<double>  state_evolution = std::vector<double>{ 0,0 };
        state_evolution.at(0) = t_curr_state.at(1);
        state_evolution.at(1) =  - (m_gravity/m_length)*sin(t_curr_state.at(0)*PI/180);
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
        std::cout << "current time =" << m_clock.getCurrTime() << '\n';
        std::cout << "state 1=" << getState().at(0) << '\n';
        std::cout << "state 2=" << getState().at(1) << '\n';
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
        gp << gp.binFile1d(theta_dot, "record") << "with lines title 'theta'" << "\n";

    }

    void Pendulum::storePlotData(std::vector<std::vector<double>> &t_plotData) {
        double t = m_clock.getCurrTime();
        std::vector<double> curr_state { t, m_state.at(0), m_state.at(1)};
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

















