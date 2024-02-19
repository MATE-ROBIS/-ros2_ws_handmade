#include "duplicate/mdrtu.hpp"

int main(){
    Mdrtu modbus1("/dev/ttyUSB0",115200,'N',8,1);
    modbus1.modbus_connection();

}



