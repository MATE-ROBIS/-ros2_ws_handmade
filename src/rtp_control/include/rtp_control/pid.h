#ifndef RTP_CONTROL_PID_H
#define RTP_CONTROL_PID_H

#include<iostream>
#include<cmath>
#include <chrono>
#include<vector>
#include "rclcpp/rclcpp.hpp"

class Pid_Controller{

    private:

    rclcpp::Clock clock;
    rclcpp::Time prev_time_;
    rclcpp::Time last_setpoint_msg_time_;
    rclcpp::Duration delta_t_;







    double control_effort_ = 0;

    double setpoint_ = 0;    
    double plant_state_;           // desired output of plant




    double error_integral_ = 0;
    double proportional_ = 0;  // proportional term of output
    double integral_ = 0;      // integral term of output
    double derivative_ = 0;    // derivative term of output

    // PID gains
    double Kp_ = 0, Ki_ = 0, Kd_ = 0;

    // Parameters for error calc. with disconinuous input
    bool angle_error_ = false;
    double angle_wrap_ = 2.0 * 3.14159;

    // Cutoff frequency for the derivative calculation in Hz.
    // Negative -> Has not been set by the user yet, so use a default.
    double cutoff_frequency_ = -1;
    
    // Setpoint timeout parameter to determine how long to keep publishing
    // control_effort messages after last setpoint message
    // -1 indicates publish indefinately, and positive number sets the timeout
  

    // Used in filter calculations. Default 1.0 corresponds to a cutoff frequency
    // at
    // 1/4 of the sample rate.
    double c_ = 1.;

    // Used to check for tan(0)==>NaN in the filter calculation
    double tan_filt_ = 1.;

    // Upper and lower saturation limits
    double upper_limit_ = 0.8, lower_limit_ = -0.8;

    // Anti-windup term. Limits the absolute value of the integral term.
    double windup_limit_ = 0.8;

    // Initialize filter data with zeros
    std::vector<double> error_, filtered_error_, error_deriv_, filtered_error_deriv_;


    int measurements_received_ = 0;
  

    public:
        Pid_Controller();
        void pid_param_init(double kp,double ki,double kd,double upper_limit,double lower_limit,double windup_limit);
        double Control_effort(double setpoint,double measured_value);

};



#endif