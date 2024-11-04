#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include <pid_controller/pid.h>

using namespace std;

double PID::calculate( double setpoint, double current )
{
    
    // Calculate error
    double error = setpoint - current;

    // Proportional term
    double Pout = Kp_ * error;

    // Integral term
    integral_ += error * dt_;
    double Iout = Ki_ * integral_;

    // Derivative term
    double derivative = (error - prev_error_) / dt_;
    double Dout = Kd_ * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > max_ )
        output = max_;
    else if( output < min_ )
        output = min_;

    // Save error to previous error
    prev_error_ = error;

    return output;
}

#endif