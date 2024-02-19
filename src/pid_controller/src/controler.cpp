#include <pid_controller/pid.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);

    auto pid =std::make_shared<PID>(5,0.1,0.1);

    rclcpp::Rate looprate(200);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(pid);
        pid->sumation();
        looprate.sleep();
    }

    rclcpp::shutdown();
    return 0;

}


