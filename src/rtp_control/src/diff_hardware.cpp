
#include "rtp_control/diff_hardware.h"

#include <cmath>



#include "hardware_interface/types/hardware_interface_type_values.hpp"

double cmd1 {0.0};
double cmd2 {0.0};

double left_out;
double right_out;



double view_p {0};
double view_i {0};
double view_d {0};
double view_imax {0};
double view_imin {0};


RtpModbus::RtpModbus(): logger_(rclcpp::get_logger("RtpModbus")){

}

return_type RtpModbus::configure(const hardware_interface::HardwareInfo & info){
    if(configure_default(info)!= return_type::OK){
        return return_type::ERROR;
    }
      RCLCPP_INFO(logger_,"--------------------");
      RCLCPP_INFO(logger_, "|RTP Configuring...|");
      RCLCPP_INFO(logger_,"--------------------");
     
      right_wheel.pid_param_init(2.0,0.00000,0.002,0.8,-0.8,0.8);
      left_wheel.pid_param_init(2.0,0.00000,0.002,0.8,-0.8,0.8);
    
    

    try {
        time_ = std::chrono::system_clock::now();

        cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
        
        cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];

        string baud_rate= (info_.hardware_parameters["baud_rate"]);  //baud_rate

        cfg_.baud_rate=atoi(baud_rate.c_str());

        string data_bit =(info_.hardware_parameters["data_bit"]);   //Data_bit

        cfg_.data_bit=atoi(data_bit.c_str());

        string stop_bit =(info_.hardware_parameters["stop_bit"]);   //Stop bit

        cfg_.stop_bit=atoi(stop_bit.c_str());

        string gear_ratio =(info_.hardware_parameters["gear_ratio"]);  //Gear_ratio

        cfg_.gear_ratio=atof(gear_ratio.c_str());

        string rps_conversion =(info_.hardware_parameters["rps_conversion"]);  //Rps Conversion

        cfg_.rps_conversion=atoi(rps_conversion.c_str());

        string port = info_.hardware_parameters["port"];           //Port selection

        cfg_.port = port.c_str();

    } 
    catch (const std::exception& e) 
    {

        // Handle the exception (print an error message, log it, etc.)

        std::cerr << "Error during parameter conversion: " << e.what() << std::endl;

        // You might want to set default values or take appropriate action to handle the error.

    }

      l_wheel_.setup(cfg_.left_wheel_name,cfg_.rps_conversion);

      r_wheel_.setup(cfg_.right_wheel_name,cfg_.rps_conversion);

      

    
      l_pid.getGains(view_p,view_i,view_d,view_imax,view_imin);
      
      con_=dual_modbus_.modbus_connection(cfg_.port,cfg_.baud_rate,cfg_.data_bit,cfg_.stop_bit);
      // con_=true;

      if (con_){

          RCLCPP_INFO(logger_,"-------------------------------------");
          RCLCPP_INFO(logger_,"| MODBUS Communication Established ! |");
          RCLCPP_INFO(logger_,"-------------------------------------");


          RCLCPP_INFO(logger_,"<------------------------------------->");
          RCLCPP_INFO(logger_,"|Port = %s",cfg_.port);
          RCLCPP_INFO(logger_,"|Baud_rate = %d",cfg_.baud_rate);
          RCLCPP_INFO(logger_,"|Data_bit = %d",cfg_.data_bit);
          RCLCPP_INFO(logger_,"|Stop_bit = %d",cfg_.stop_bit);
          RCLCPP_INFO(logger_,"|Gear_ratio = %d",cfg_.gear_ratio);
          RCLCPP_INFO(logger_,"|RPS_Conversion = %d",cfg_.rps_conversion);
          RCLCPP_INFO(logger_,"|Left_wheel_name = %s",cfg_.left_wheel_name.c_str());
          RCLCPP_INFO(logger_,"|Right_Wheel_name = %s",cfg_.right_wheel_name.c_str());
          RCLCPP_INFO(logger_,"|P = %f",view_p);
          RCLCPP_INFO(logger_,"|I = %f",view_i);
          RCLCPP_INFO(logger_,"|D = %f",view_d);
          RCLCPP_INFO(logger_,"<------------------------------------->");

      }
      else{

          RCLCPP_INFO(logger_,"------------------------------------------");
          RCLCPP_INFO(logger_,"| MODBUS Communication Not Established ! |");
          RCLCPP_INFO(logger_,"------------------------------------------");


          RCLCPP_INFO(logger_,"<------------------------------------->");
          RCLCPP_INFO(logger_,"|Port = %s",cfg_.port);
          RCLCPP_INFO(logger_,"|Baud_rate = %d",cfg_.baud_rate);
          RCLCPP_INFO(logger_,"|Data_bit = %d",cfg_.data_bit);
          RCLCPP_INFO(logger_,"|Stop_bit = %d",cfg_.stop_bit);
          RCLCPP_INFO(logger_,"|Gear_ratio = %d",cfg_.gear_ratio);
          RCLCPP_INFO(logger_,"|RPS_Conversion = %d",cfg_.rps_conversion);
          RCLCPP_INFO(logger_,"|Left_wheel_name = %s",cfg_.left_wheel_name.c_str());
          RCLCPP_INFO(logger_,"|Right_Wheel_name = %s",cfg_.right_wheel_name.c_str());
          RCLCPP_INFO(logger_,"<------------------------------------->");
          

      }
      RCLCPP_INFO(logger_,"----------------------------");
      RCLCPP_INFO(logger_,"|RTP Configuration_Finished| ");
      RCLCPP_INFO(logger_,"----------------------------");
      status_=hardware_interface::status::CONFIGURED;
      return return_type::OK;

}


std::vector<hardware_interface::StateInterface> RtpModbus::export_state_interfaces()
{
  // We need to set up a velocity interface for each wheel

  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_POSITION, &r_wheel_.pos));

  state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_POSITION, &l_wheel_.pos));

  RCLCPP_INFO(logger_,"----------------------------");
  for (const auto& state_interface : state_interfaces) {
    RCLCPP_INFO(logger_, "State Interface: Name=%s, Type=%s",
                state_interface.get_name().c_str(), state_interface.get_interface_name().c_str());
  }
  


  return state_interfaces;
}







std::vector<hardware_interface::CommandInterface> RtpModbus::export_command_interfaces()
{
 

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd));


  for (const auto& command_interface : command_interfaces) {
    RCLCPP_INFO(logger_, "Command Interface: Name=%s, Type=%s ",
                command_interface.get_name().c_str(), command_interface.get_interface_name().c_str());
  }
  

  return command_interfaces;
}

return_type RtpModbus::start()
{
  RCLCPP_INFO(logger_,"-----------------------");
  RCLCPP_INFO(logger_,"| Starting Controller... |");
  RCLCPP_INFO(logger_,"-----------------------");

  status_ = hardware_interface::status::STARTED;

  return return_type::OK;
}


return_type RtpModbus::stop()
{
  RCLCPP_INFO(logger_,"-----------------------");
  RCLCPP_INFO(logger_,"|Stopping Controller...|");
  RCLCPP_INFO(logger_,"-----------------------");
  status_ = hardware_interface::status::STOPPED;

  return return_type::OK;
}

hardware_interface::return_type RtpModbus::read()
{

  // TODO fix chrono duration

  delta=clock.now();


  r_wheel_.rps=dual_modbus_.read_registor(16,1,1); //slave1
  l_wheel_.rps=dual_modbus_.read_registor(16,1,2)*-1; //slave2

  

  double linear {0.0};
  double angular {0.0};
  double cirumference = (2*M_PI*0.08);

                                                                                                                                                                                                                        // linear=(((r_wheel_.rps + l_wheel_.rps)/2));
                                                                                                                                                                                                                        // angular=(((r_wheel_.rps-l_wheel_.rps)/0.785));
  double Actual_right_rps;
  double Actual_left_rps;      


  Actual_right_rps=l_wheel_.rps*cirumference;
  Actual_left_rps=r_wheel_.rps*cirumference;




  l_wheel_.vel=left_wheel.Control_effort(l_wheel_.rps,Actual_left_rps);
  r_wheel_.vel=right_wheel.Control_effort(r_wheel_.cmd,Actual_right_rps);









                                                                                                                                                                                          // l_wheel_.vel=((l_wheel_.rps/cfg_.gear_ratio)/cfg_.rps_conversion);
                                                                                                                                                                                          // r_wheel_.vel=((r_wheel_.rps/cfg_.gear_ratio)/cfg_.rps_conversion);

 
  




  // RCLCPP_INFO(logger_,"%f",delta.now());

 



  return return_type::OK;

  
}

hardware_interface::return_type RtpModbus::write()
{

  if (!con_)
  {
    return return_type::ERROR;
  }

  

  int r_wheel_modbus;
  int l_wheel_modbus;

  // r_wheel_modbus=static_cast<double>(r_wheel_.cmd*cfg_.gear_ratio*cfg_.rps_conversion);
  // l_wheel_modbus=static_cast<double>(l_wheel_.cmd*cfg_.gear_ratio*cfg_.rps_conversion);

  //pid out
  r_wheel_modbus=static_cast<double>(r_wheel_.vel*cfg_.gear_ratio*cfg_.rps_conversion);
  l_wheel_modbus=static_cast<double>((-1*l_wheel_.vel)*cfg_.gear_ratio*cfg_.rps_conversion);

  // RCLCPP_INFO(logger_,"Right_wheel_cmd = %d",l_wheel_modbus);
  // RCLCPP_INFO(logger_,"Right_wheel_cmd = %d",r_wheel_modbus);

  // if(l_wheel_modbus>0 && r_wheel_modbus>0 ){

  //   dual_modbus_.write_registor(366,2,r_wheel_modbus,1);
  //   dual_modbus_.write_registor(366,2,-abs(l_wheel_modbus),2);
  //   RCLCPP_INFO(logger_,"Front");

    

  // }
  // else if (r_wheel_modbus<0 && l_wheel_modbus<0)
  // {
  //   dual_modbus_.write_registor(366,2,r_wheel_modbus,1);
  //   dual_modbus_.write_registor(366,2,abs(l_wheel_modbus),2);
  //   RCLCPP_INFO(logger_,"Back");
    
    
  // }
  // else if (r_wheel_modbus>0 == -l_wheel_modbus<0)
  // {
  //   dual_modbus_.write_registor(366,2,r_wheel_modbus,1);
  //   dual_modbus_.write_registor(366,2,-abs(l_wheel_modbus),2);
  //   RCLCPP_INFO(logger_,"left");
   
    
  // }
  // else if (-r_wheel_modbus<0 == l_wheel_modbus>0)
  // {
  //   dual_modbus_.write_registor(366,2,r_wheel_modbus,1);
  //   dual_modbus_.write_registor(366,2,abs(l_wheel_modbus),2);
  //   RCLCPP_INFO(logger_,"right");
    
    
  // }
  // else if (r_wheel_modbus==0 && l_wheel_modbus==0)
  // {
  //   dual_modbus_.write_registor(366,2,r_wheel_modbus,1);
  //   dual_modbus_.write_registor(366,2,abs(l_wheel_modbus),2);
    
    
  // }

    dual_modbus_.write_registor(366,2,r_wheel_modbus,1);
    dual_modbus_.write_registor(366,2,l_wheel_modbus,2);
  
  
  


 




  return return_type::OK;


  
}


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(RtpModbus,hardware_interface::SystemInterface)