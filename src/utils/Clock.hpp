// Created by Edoardo Ghini on 14/06/2018.

#pragma once

namespace dyn_modeling {


    class Clock {

    private:
        double m_curr_time;
        double m_time_step;

    public:
        Clock( double t_time_step);
        inline double getCurrTime() const { return m_curr_time; }
        inline void setCurrTime( double t_new_time) { m_curr_time= t_new_time; }

        void thick();

    };

}
