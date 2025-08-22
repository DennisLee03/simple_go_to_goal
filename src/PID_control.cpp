#include "demo_diff_drive_pkg/PID_control.hpp"
PID_control::PID_control(double kp, double ki, double kd, double max_integral, double max_output)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->max_output = max_output;
    this->max_integral = max_integral;
    this->prev_time = std::chrono::steady_clock::now();
    
    this->error = 0;
    this->prev_error = 0;
    this->integral = 0;
    this->output = 0;
}

double PID_control::pid_calc(double target, double response)
{
    // dt
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> dt_duration = current_time - this->prev_time;
    double dt = dt_duration.count();
    this->prev_time = current_time;

    // e(t)
    this->prev_error = this->error;
    this->error = target - response;

    // Kp*e(t)
    double p_out = this->kp*this->error;

    // Ki*[e(t)dt] with limit
    this->integral += this->ki*this->error*dt;
    if(this->integral > this->max_integral) this->integral = this->max_integral;
    else if(this->integral < -this->max_integral) this->integral = -this->max_integral;

    // Kd*[e(t)/dt]
    double d_out = dt ? (this->kd*(this->error-this->prev_error)/dt) : 0.0;

    this->output = p_out + this->integral + d_out;
    if(this->output > this->max_output) this->output = this->max_output;
    else if(this->output < -this->max_output) this->output = -this->max_output;
    
    return this->output;
}