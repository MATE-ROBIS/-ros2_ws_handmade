#ifndef RTP_CONTROL_RTP_MODBUS_H
#define RTP_CONTROL_ARDUINO_RTP_MODBUS_H

#include <modbus/modbus.h>
#include <modbus/modbus-rtu.h>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>


using namespace std;
class Rtp_Modbus_Dual{
    public:


        Rtp_Modbus_Dual();

        bool modbus_connection(const char* port,int baud_rate,int data_bit,int stop_bit);

        double read_registor(int addr,int num_reg,int salve);

        int write_registor(int write_addr,int num_reg,int write_value,int slave);

        void Emergency_stop(int op_addr,int num_reg,uint16_t write_value,int slave );



    private:
        modbus_t *ctx;
        const char* port_ ;
        int baud_rate_;
        
        int data_bit_;
        int stop_bit_;




};





#endif