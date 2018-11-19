// Created by dinies on 11/10/2018.

#pragma once

namespace dyn_modeling {


    class ClockInterface {

    protected:
        double m_curr_time = 0;
        double m_delta_t = 0.001;

    public:
	ClockInterface() = default;

        explicit ClockInterface(const double t_time_step):
		m_delta_t( t_time_step)
	    {};

        virtual double getDeltaT() =0;
        virtual double getCurrTime() =0;
        virtual void setCurrTime(const double t_new_time) = 0;
        virtual void tick() =0;

	virtual ~ClockInterface() = default;

    };

}
