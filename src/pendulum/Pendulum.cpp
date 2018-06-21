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
    m_gravity(9.80665),
    m_current_input(0),
    m_controller( Controller(t_gains)),
    m_pixel_height(600),
    m_pixel_width(960),
    m_static_drawn_flag(false)
  {
    m_drawing.create(600,960);
    m_drawing= cv::Vec3b(255, 255, 255);
  }


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

  double Pendulum::computeGravityCompens( const double t_theta_ref) {
    return  m_gravity * sin( t_theta_ref) / m_length;
  }

  void Pendulum::staticDraw(){
    cv::namedWindow("Pendulum");
    int horizon_y = (int)(m_pixel_height/3);
    cv::Point2d h1( 1, horizon_y );
    cv::Point2d h2( m_pixel_width, horizon_y);
    cv::Scalar green(0,255,20);
    cv::line( m_drawing, h1,h2,green);
    m_static_drawn_flag = true;
  }


  void Pendulum::dynamicDraw(cv::Point2d &t_prev_head){
    const cv::Point2d foot( (int)(m_pixel_width/2),(int)(m_pixel_height/3));

    // cv::Point2d prev_head= t_prev_head;

    cv::Scalar white(255,255,255);
    cv::Scalar dark_red(20,0,255);

    cv::line( m_drawing, foot,t_prev_head,white);

    const double scale = 50;
    Eigen::Vector2d origin_p2_w(0 , - scale* m_length );
    Eigen::Matrix2d R;
    R << cos( m_state(0)), -sin(m_state(0)),
      sin(m_state(0)), cos(m_state(0));

    Eigen::Vector2d p2_w = R * origin_p2_w;
    double p2_c_x = foot.x + p2_w(0);
    double p2_c_y = foot.y - p2_w(1);

    cv::Point2d head( p2_c_x, p2_c_y);
    cv::line( m_drawing, foot,head,dark_red);
    cv::imshow("Pendulum", m_drawing);
    cv::waitKey(1);
    t_prev_head = head;
    // std::cout << ", \n" << m_state(0) << ", " << p2_w(0)<<", " << p2_w(1) << ", "<< p2_c_x << ", " << p2_c_y <<"\n";
  }

  void Pendulum::drawImg(cv::Point2d &t_prev_head){
    if (m_static_drawn_flag){
      dynamicDraw( t_prev_head);
    }
    else {
      staticDraw();
      dynamicDraw( t_prev_head);
    }
  }

  void Pendulum::cycle(const int t_numCycles, double t_theta_ref, bool t_drawing_flag = false) {
    std::vector < std::vector<double>> plotting_data;
    Eigen::Vector2d evolution_vec;
    double g_comp = computeGravityCompens( t_theta_ref);
    cv::Point2d prev_point(1,1);

    for (int i = 0; i < t_numCycles; ++i) {
      std::vector<double> controller_output = m_controller.computeInput(t_theta_ref,m_state,g_comp);
      setCurrInput( controller_output.at(0));
      updateState();
      storePlotData( plotting_data, controller_output.at(1));

      if(t_drawing_flag && i%5==0){
        drawImg(prev_point);
      }
    }
    plotStateCycle(plotting_data);
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


