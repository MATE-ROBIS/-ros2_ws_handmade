#ifndef RTP_CONTROL_RTPMODBUS_H
#define RTP_CONTROL_RTPMODBUS_H

#include <cstring>
#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"

#include "config.h"
#include "wheel.h"
#include "rtp_modbus.h"
#include "pid.h"
#include <control_toolbox/pid.hpp>
#include <control_toolbox/control_toolbox/filters.hpp>
#include <control_toolbox/pid_ros.hpp>




using hardware_interface::return_type;

class RtpModbus : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{


public:
   
    RtpModbus();
    
    return_type configure(const hardware_interface::HardwareInfo & info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    return_type start() override;

    return_type stop() override;

    return_type read() override;

    return_type write() override;



private:

    Config cfg_;
    
    Rtp_Modbus_Dual dual_modbus_;

    Wheel l_wheel_;
    Wheel r_wheel_;



    rclcpp::Logger logger_;

    bool con_;

    

    std::chrono::time_point<std::chrono::system_clock> time_;

    rclcpp::Clock clock;
    
    

    Pid_Controller right_wheel;
    Pid_Controller left_wheel;

    // Pid_Controller left_wheel;
    control_toolbox::Pid l_pid;
    
    control_toolbox::Pid r_pid;

    rclcpp::Time delta;
    
    
    
    
  
};




#endif // DIFFDRIVE_ARDUINO_REAL_ROBOT_H