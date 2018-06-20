//
// Created by Edoardo Ghini on 13/06/2018.
//

#include "Pendulum.hpp"

namespace dyn_modeling {

    Pendulum::Pendulum(
            const Eigen::Vector2d t_initial_state,
            const double t_delta_t,
            const double t_length,
            const double t_mass,
            const std::vector<double> t_gains):
            m_state( t_initial_state),
            m_clock( Clock(t_delta_t)),
            m_length( t_length),
            m_mass( t_mass),
            m_gravity(9.81),
            m_current_input(0),
            m_controller( Controller(t_gains))
            {}


    void Pendulum::operator()(const dyn_modeling::Pendulum::state_type &x, dyn_modeling::Pendulum::state_type &dxdt,
                              const double /* t */ ) {
        dxdt[0] = x[1];
        dxdt[1] = - (m_gravity/m_length)*sin(x[0]) + m_current_input ;
    }


    void Pendulum::updateState() {
        Eigen::Vector2d oldState = getState();
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

    void Pendulum::storePlotData(std::vector<std::vector<double>> &t_plotData, double t_error ) {
        double t = m_clock.getCurrTime();
        const double energy = this->computeEnergy();
        std::vector<double> curr_data{ t, m_state(0), m_state(1),m_current_input,t_error, energy};
        t_plotData.push_back(curr_data);
    }

    double Pendulum::computeEnergy() {
        const double h = m_length* (1 - cos(m_state(0))) ;
        const double potential = m_mass * m_gravity * h;
        const double v = m_state(1) * m_length;
        const double kinetic =  m_mass * v * v * 1/2;
        return kinetic + potential;
    }

  void Pendulum::cycle(const int t_numCycles, double t_theta_ref) {
        std::vector < std::vector<double>> plotting_data;
        Eigen::Vector2d evolution_vec;
        for (int i = 0; i < t_numCycles; ++i) {
            std::vector<double> controller_output = m_controller.computeInput( t_theta_ref, m_state);
            setCurrInput( controller_output.at(0));
            this->updateState();
            this->storePlotData( plotting_data, controller_output.at(1));
        }
        this->plotStateCycle(plotting_data);
    }

    void Pendulum::plotStateCycle(std::vector<std::vector<double>> &t_plotData) {

        std::vector< boost::tuple<double,double>> theta;
        std::vector< boost::tuple<double,double>> theta_dot;
        std::vector< boost::tuple<double,double>> input;
        std::vector< boost::tuple<double,double>> error;
        std::vector< boost::tuple<double,double>> energy;
        for ( auto vec : t_plotData){
            theta.push_back( boost::make_tuple( vec.at(0),vec.at(1)));
            theta_dot.push_back( boost::make_tuple( vec.at(0),vec.at(2)));
            input.push_back( boost::make_tuple( vec.at(0),vec.at(3)));
            error.push_back( boost::make_tuple( vec.at(0),vec.at(4)));
            energy.push_back( boost::make_tuple( vec.at(0),vec.at(5)));
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
        gp << gp.binFile1d(input, "record") << "with lines title 'input'" << "\n";
        gp << "set terminal qt 4\n";
        gp << "plot";
        gp << gp.binFile1d(error, "record") << "with lines title 'error'" << "\n";
        gp << "set terminal qt 5\n";
        gp << "plot";
        gp << gp.binFile1d(energy, "record") << "with lines title 'energy'" << "\n";

    }

}


