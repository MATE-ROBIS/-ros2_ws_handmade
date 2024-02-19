#include <pid_controller/pid.hpp>


PID::PID(double Kp, double Ki, double Kd) : Node("Pid_Controller"){
    kp_=Kp;
    kd_=Kd;
    ki_=Ki;
    auto twits_callback =[this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void
    {
        set_point_=msg->linear.x;
        

     
    };

    set_point_sub_=create_subscription<geometry_msgs::msg::Twist>("/diff_controller/cmd_vel_unstamped",twits_callback);

}
