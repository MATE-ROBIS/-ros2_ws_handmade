#ifndef RTP_CONTROL_CONFIG_H
#define RTP_CONTROL_CONFIG_H

#include <string>


struct Config
{
  std::string left_wheel_name = "left_wheel_joint";
  std::string right_wheel_name = "right_wheel_joint";
  
  float loop_rate = 30;

  const char *port = "/dev/ttyUSB0";
  int baud_rate = 115200;
  const char parity='N';
  int data_bit = 8;
  int stop_bit = 1;

  int rps_conversion = 240;
  int gear_ratio=15;
  

};


#endif // DIFFDRIVE_ARDUINO_CONFIG_H