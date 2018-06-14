// Created by Edoardo Ghini on 14/06/2018.

#include "Clock.hpp"

namespace dyn_modeling {
    Clock::Clock(double t_time_step): m_curr_time(0), m_time_step(t_time_step){
    }

    void Clock::thick() {
        double t = getCurrTime() + m_time_step;
        setCurrTime(t);
    }

}
