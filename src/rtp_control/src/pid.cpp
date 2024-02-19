#include "rtp_control/pid.h"

void Pid_Controller::pid_param_init(double kp,double ki,double kd,double upper_limit,double lower_limit,double windup_limit){
  Kp_=kp;
  Kd_=Kd_;
  Ki_=ki;
  upper_limit_=upper_limit;
  lower_limit_=lower_limit;
  windup_limit_=windup_limit;
}

double Pid_Controller::Control_effort(double setpoint,double measured_value){

    error_.at(2) = error_.at(1);
    error_.at(1) = error_.at(0);
    error_.at(0) = setpoint_ - plant_state_;


     while (error_.at(0) > angle_wrap_ / 2.0)
      {
        error_.at(0) -= angle_wrap_;

        // The proportional error will flip sign, but the integral error
        // won't and the filtered derivative will be poorly defined. So,
        // reset them.
        error_deriv_.at(2) = 0.;
        error_deriv_.at(1) = 0.;
        error_deriv_.at(0) = 0.;
        error_integral_ = 0.;
      }

    if (!(prev_time_.nanoseconds()/1e9)==0)  // Not first time through the program
    {
      delta_t_ = clock.now() - prev_time_;
      prev_time_ = clock.now();
      if (0 == (delta_t_.nanoseconds()/1e9))
      {
      
        return;
      }
    }
    else
    {
     
      prev_time_ = clock.now();
      return;
    }

        // integrate the error
    error_integral_ += error_.at(0) * (delta_t_.nanoseconds()/1e9);

    // Apply windup limit to limit the size of the integral term
    if (error_integral_ > fabsf(windup_limit_))
      error_integral_ = fabsf(windup_limit_);

    if (error_integral_ < -fabsf(windup_limit_))
      error_integral_ = -fabsf(windup_limit_);

    // My filter reference was Julius O. Smith III, Intro. to Digital Filters
    // With Audio Applications.
    // See https://ccrma.stanford.edu/~jos/filters/Example_Second_Order_Butterworth_Lowpass.html
    if (cutoff_frequency_ != -1)
    {
      // Check if tan(_) is really small, could cause c = NaN
      tan_filt_ = tan((cutoff_frequency_ * 6.2832) * (delta_t_.nanoseconds()/1e9) / 2);

      // Avoid tan(0) ==> NaN
      if ((tan_filt_ <= 0.) && (tan_filt_ > -0.01))
        tan_filt_ = -0.01;
      if ((tan_filt_ >= 0.) && (tan_filt_ < 0.01))
        tan_filt_ = 0.01;

      c_ = 1 / tan_filt_;
    }

    filtered_error_.at(2) = filtered_error_.at(1);
    filtered_error_.at(1) = filtered_error_.at(0);
    filtered_error_.at(0) = (1 / (1 + c_ * c_ + 1.414 * c_)) * (error_.at(2) + 2 * error_.at(1) + error_.at(0) -
                                                                (c_ * c_ - 1.414 * c_ + 1) * filtered_error_.at(2) -
                                                                (-2 * c_ * c_ + 2) * filtered_error_.at(1));

    // Take derivative of error
    // First the raw, unfiltered data:
    error_deriv_.at(2) = error_deriv_.at(1);
    error_deriv_.at(1) = error_deriv_.at(0);
    error_deriv_.at(0) = (error_.at(0) - error_.at(1)) / (delta_t_.nanoseconds()/1e9);

    filtered_error_deriv_.at(2) = filtered_error_deriv_.at(1);
    filtered_error_deriv_.at(1) = filtered_error_deriv_.at(0);

    filtered_error_deriv_.at(0) =
        (1 / (1 + c_ * c_ + 1.414 * c_)) *
        (error_deriv_.at(2) + 2 * error_deriv_.at(1) + error_deriv_.at(0) -
         (c_ * c_ - 1.414 * c_ + 1) * filtered_error_deriv_.at(2) - (-2 * c_ * c_ + 2) * filtered_error_deriv_.at(1));

    // calculate the control effort
    proportional_ = Kp_ * filtered_error_.at(0);
    integral_ = Ki_ * error_integral_;
    derivative_ = Kd_ * filtered_error_deriv_.at(0);
    control_effort_ = proportional_ + integral_ + derivative_;

    // Apply saturation limits
    if (control_effort_ > upper_limit_)
      control_effort_ = upper_limit_;
    else if (control_effort_ < lower_limit_)
      control_effort_ = lower_limit_;


  

}


