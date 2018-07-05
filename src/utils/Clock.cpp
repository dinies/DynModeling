// Created by dinies on 14/06/2018.

#include "Clock.hpp"

namespace dyn_modeling {
    Clock::Clock(double t_delta_t): m_curr_time(0), m_delta_t(t_delta_t){
    }

    void Clock::tick() {
        double t = getCurrTime() + m_delta_t;
        setCurrTime(t);
    }

}
