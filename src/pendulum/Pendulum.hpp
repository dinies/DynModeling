// Created by Edoardo Ghini on 13/06/2018.

#pragma once
#include <Eigen/Core>
//#include <Eigen/Geometry>
//#include <Eigen/StdVector>
#include <unistd.h>
#include <boost/serialization/array_wrapper.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/numeric/odeint.hpp>
#include <iostream>
#include <vector>
#include <math.h>
#include "opencv2/opencv.hpp"
#include "../utils/Clock.hpp"
#include "../../include/gnuplot-iostream.h"
#include "../controllers/Controller.hpp"

namespace dyn_modeling {
  class Pendulum {
    typedef cv::Mat_< cv::Vec3b > RGBImage;

  private:
    Eigen::Vector2d m_state;
    Clock m_clock;
    double m_length;
    double m_mass;
    double m_gravity;
    double m_current_input;
    Controller m_controller;

    int m_pixel_height;
    int m_pixel_width;
    bool m_static_drawn_flag;
    RGBImage m_drawing;


    void updateState();

    void printState();

    void plotStateCycle(std::vector< std::vector<double>>& t_plotData);

    void storePlotData(std::vector< std::vector<double>>& t_plotData, double t_error);

    double computeEnergy();

    inline void setCurrInput(double t_input) { m_current_input= t_input; }

    double computeGravityCompens( double t_theta_ref);


  public:

    typedef std::vector< double > state_type;

    Pendulum(Eigen::Vector2d t_initial_state, double t_delta_t, double t_length, double t_mass, std::vector<double> t_gains );

    inline const Eigen::Vector2d getState() const { return m_state; }

    inline void setState(Eigen::Vector2d &t_newState) { m_state = t_newState; }

    inline double getCurrInput() { return m_current_input; }

    void cycle(int t_numCycles, double t_reference, bool t_drawing_flag);

    void operator() ( const state_type &x , state_type &dxdt , double /* t */ );

  };
}

