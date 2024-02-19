#ifndef RTP_CONTROL_WHEEL_H
#define RTP_CONTROL_WHEEL_H

#include <string>
#include "std_msgs/msg/float64.hpp"


class Wheel
{
    public:

    std::string name = "";
    double enc  {0};
    double cmd=0;
    
    double vel {0};

    double pos {0};

    double velSetPt = 0;

    double rps {0};

    Wheel()=default;

    Wheel(const std::string &wheel_name, double rps);
    
    void setup(const std::string &wheel_name, double rps);

    double actual_rps();



};


#endif // DIFFDRIVE_ARDUINO_WHEEL_H