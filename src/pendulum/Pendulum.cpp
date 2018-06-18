//
// Created by Edoardo Ghini on 13/06/2018.
//

#include "Pendulum.hpp"


#define PI 3.14159265

namespace dyn_modeling {

    Pendulum::Pendulum(
            const Eigen::Vector2d t_initial_state,
            const double t_delta_t,
            const double t_length,
            const double t_mass):
            m_state( t_initial_state),
            m_clock( Clock(t_delta_t)),
            m_length( t_length),
            m_mass( t_mass),
            m_gravity(9.81)
            {}


    void Pendulum::operator()(const dyn_modeling::Pendulum::state_type &x, dyn_modeling::Pendulum::state_type &dxdt,
                              const double /* t */ ) {
        dxdt[0] = x[1];
        dxdt[1] = - (m_gravity/m_length)*sin(x[0]*PI/180);
    }


    void Pendulum::updateState() {
        Eigen::Vector2d oldState = getState();
        // when i will implement the controller , the current input will be added as class member m_curr_input
        // and taken from the overloaded operator function with this.m_curr_input
        state_type x(2);
        x[0] = oldState(0);
        x[1] = oldState(1);
        double t_0 = m_clock.getCurrTime();
        double t_f = t_0 + m_clock.getDeltaT();
        double initial_step_size = (t_f-t_0)/100;
        boost::numeric::odeint::integrate( boost::ref( *this) ,x, t_0, t_f,initial_step_size );
        m_clock.thick();
        Eigen::Vector2d new_state( x[0], x[1]);
        this->setState(new_state);
    }


    void Pendulum::printState() {
        std::cout << "current time =" << m_clock.getCurrTime() << '\n';
        std::cout << "state 1=" << getState()(0) << '\n';
        std::cout << "state 2=" << getState()(1) << '\n';
    }

    void Pendulum::storePlotData(std::vector<std::vector<double>> &t_plotData, double t_curr_input) {
        double t = m_clock.getCurrTime();
        const double e = this->computeEnergy();
        std::vector<double> curr_data{ t, m_state(0), m_state(1),t_curr_input,e};
        t_plotData.push_back(curr_data);
    }

    double Pendulum::computeEnergy() {
        const double theta_zero = 0;
        const double one  = cos(theta_zero*PI/180);
        const double h_offset = cos(m_state(0)*PI/180);
        const double delta_theta = one  - h_offset;
        const double h = m_length* delta_theta ;
        const double potential = m_mass * m_gravity * h;
        const double kinetic =  m_mass * m_state(1)* m_state(1) * 1/2;
        std::cout << "debug "<< h << ", "   <<m_mass << ", " << potential << ", " << kinetic << "\n";
        return kinetic + potential;
    }

    void Pendulum::cycle(const int t_numCycles) {
        std::vector < std::vector<double>> plotting_data;
        Eigen::Vector2d evolution_vec;
        for (int i = 0; i < t_numCycles; ++i) {
            this->updateState();
            double curr_input = 0;//TODO implement controller
            this->storePlotData( plotting_data, curr_input);
        }
        this->plotStateCycle(plotting_data);
    }

    void Pendulum::plotStateCycle(std::vector<std::vector<double>> &t_plotData) {

        std::vector< boost::tuple<double,double>> theta;
        std::vector< boost::tuple<double,double>> theta_dot;
        std::vector< boost::tuple<double,double>> theta_dot_dot;
        std::vector< boost::tuple<double,double>> energy;
        for ( auto vec : t_plotData){
            theta.push_back( boost::make_tuple( vec.at(0),vec.at(1)));
            theta_dot.push_back( boost::make_tuple( vec.at(0),vec.at(2)));
            theta_dot_dot.push_back( boost::make_tuple( vec.at(0),vec.at(3)));
            energy.push_back( boost::make_tuple( vec.at(0),vec.at(4)));
        }
        Gnuplot gp;
        gp << "set terminal qt 1\n";
        gp << "plot";
        gp << gp.binFile1d(theta, "record") << "with lines title 'theta'" << "\n";
        gp << "set terminal qt 2\n";
        gp << "plot";
        gp << gp.binFile1d(theta_dot, "record") << "with lines title 'theta dot'" << "\n";
        gp << "set terminal qt 3\n";
        gp << "plot";
        gp << gp.binFile1d(theta_dot_dot, "record") << "with lines title 'theta dot_dot'" << "\n";
        gp << "set terminal qt 4\n";
        gp << "plot";
        gp << gp.binFile1d(energy, "record") << "with lines title 'energy'" << "\n";

    }

}


