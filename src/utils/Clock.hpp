// Created by dinies on 14/06/2018.

#pragma once

namespace dyn_modeling {


    class Clock {

    private:
        double m_curr_time;
        double m_delta_t;

    public:
        explicit Clock( double t_time_step);
        inline double getDeltaT() const { return m_delta_t; }
        inline double getCurrTime() const { return m_curr_time; }
        inline void setCurrTime( double t_new_time) { m_curr_time= t_new_time; }

        void tick();

    };

}
