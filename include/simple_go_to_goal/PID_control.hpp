#ifndef PID_CONTROL_HPP
#define PID_CONTROL_HPP

#include <chrono>

class PID_control
{
    public:
        PID_control(double kp, double ki, double kd, double max_integral, double max_output);

        double pid_calc(double target, double response);

    private:
        double kp, ki, kd;
        double error, prev_error;
        double integral, max_integral;
        double output, max_output;
        std::chrono::steady_clock::time_point prev_time;
};

#endif // PID_CONTROL_HPP