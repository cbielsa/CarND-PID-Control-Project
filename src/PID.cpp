#include "PID.h"
#include <iostream>

using namespace std;


PID::PID()
  : p_error_(0.), i_error_(0.), d_error_(0.),
    Kp_(0.), Ki_(0.), Kd_(0.)
{}


PID::~PID()
{}


void PID::Init(double Kp, double Ki, double Kd) {
	p_error_ = 0.;
	i_error_ = 0.;
	d_error_ = 0.;
	Kp_ = Kp;
	Ki_ = Ki;
	Kd_ = Kd;
	return; 
}


void PID::UpdateError(double cte) {
	// assume unit delta-time
	d_error_ = cte-p_error_;
	p_error_ = cte;
	i_error_ += cte;
	return;
}


double PID::Actuation() const {
	return -Kp_*p_error_ -Ki_*i_error_ -Kd_*d_error_;
}

