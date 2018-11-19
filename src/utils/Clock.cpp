// Created by dinies on 14/06/2018.

#include "Clock.hpp"

namespace dyn_modeling {

    void Clock::tick() {
        double t = getCurrTime() + m_delta_t;
        setCurrTime(t);
    }

}
