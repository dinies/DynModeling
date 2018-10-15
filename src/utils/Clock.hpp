// Created by dinies on 14/06/2018.

#pragma once

#include "ClockInterface.hpp"

namespace dyn_modeling {


  class Clock: public ClockInterface {

    public:
      explicit Clock(const double t_time_step):
        ClockInterface( t_time_step)
    {};
      inline double getDeltaT(){ return m_delta_t; }
      inline double getCurrTime(){ return m_curr_time; }
      inline void setCurrTime(const double t_new_time) { m_curr_time= t_new_time; }

      void tick();
  };
}
