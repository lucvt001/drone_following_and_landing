#ifndef _PID_H_
#define _PID_H_

class PID
{
public:
    // Kp -  proportional gain
    // Ki -  Integral gain
    // Kd -  derivative gain
    // dt -  loop interval time
    // max - maximum value of manipulated variable
    // min - minimum value of manipulated variable
    // PID( double Kp, double Ki, double Kd, double dt, double max, double min );
    PID ( double Kp, double Ki, double Kd, double dt, double max, double min )
        : Kp_(Kp), Ki_(Ki), Kd_(Kd), dt_(dt), max_(max), min_(min), prev_error_(0.0), integral_(0.0) {};

    // Returns the manipulated variable given a setpoint and current process value
    double calculate( double setpoint, double current );

private:
    double Kp_;
    double Ki_;
    double Kd_;
    double dt_;
    double max_;
    double min_;
    double prev_error_;
    double integral_;
};

#endif