#include "rtp_control/wheel.h"

#include <cmath>


Wheel::Wheel(const std::string &wheel_name, double rps)
{
  setup(wheel_name, rps);
}


void Wheel::setup(const std::string &wheel_name, double rps)
{
  name = wheel_name;
  rps=rps;
}

double Wheel::actual_rps()
{
  return enc / rps;
}