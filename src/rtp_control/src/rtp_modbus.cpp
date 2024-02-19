#include "rtp_control/rtp_modbus.h"

#include "rclcpp/rclcpp.hpp"


Rtp_Modbus_Dual::Rtp_Modbus_Dual(){
    cout<<"Modbus_constructor is called"<<endl;

}

bool Rtp_Modbus_Dual::modbus_connection(const char* port,int baud_rate,int data_bit,int stop_bit){
    ctx=modbus_new_rtu("/dev/ttyUSB0", 115200,'N', 8, 1);

    if(modbus_connect(ctx)==-1){

        cerr<<"Context is not created! Error_id"<<modbus_strerror(errno)<<endl;

        return false;
    }

    else

    {
        cout<<"------------------------------------"<<endl;
        cout<<"| MODBUS Communication Established |"<<endl;
        cout<<"------------------------------------"<<endl;

    }

    return true;

}

double Rtp_Modbus_Dual::read_registor(int addr,int num_reg,int salve){
    modbus_set_debug(ctx, true);

    uint16_t value[1];

    modbus_set_slave(ctx,salve);

    uint16_t result=modbus_read_registers(ctx,16,num_reg,value);

    int16_t convert=static_cast<int16_t>(value[0]);

    double drps=static_cast<double>(convert);
    double Mechanical_reduction=drps/240;
    double actual_rps=Mechanical_reduction/15;
    std::cout << actual_rps << std::endl;


    if(result==-1)
    {
        cout<<"Read value not read Slave no "<<salve<<endl;
    }
    else
    {
        cout<<"Read value read on Slave no "<<salve<<endl;
    }
    return actual_rps;
}


int Rtp_Modbus_Dual::write_registor(int write_addr, int num_reg, int write_value, int slave) {
    modbus_set_slave(ctx,slave);
    modbus_set_debug(ctx, true);


    uint16_t Lower_part=static_cast<uint16_t>(write_value & 0xFFFFF);
    uint16_t Higher_part=static_cast<uint16_t>((write_value>>16)& 0xFFFFF);
   
    uint16_t reg_val[64]={Higher_part,Lower_part};

    auto result= modbus_write_registers(ctx,write_addr,num_reg,reg_val);

    if(result==-1)
    {
        cout<<"Write value is not loaded on slave no "<<slave<<endl;
        return 0;
    }
    else
    {
        cout<<"Write value is written on slave no "<<slave<<endl;
        return 1;
    }




}

void Rtp_Modbus_Dual::Emergency_stop(int op_addr, int num_reg, uint16_t write_value, int slave) {
    modbus_set_slave(ctx,slave);
    modbus_set_debug(ctx, true);

    uint16_t Lower_part=static_cast<uint16_t>(write_value & 0xFFFFF);
    uint16_t Higher_part=static_cast<uint16_t>((write_value>>16)& 0xFFFFF);

    uint16_t reg_val[2]={Higher_part,Lower_part};

    auto res = modbus_write_registers(ctx,op_addr,num_reg,reg_val);

    if(res==-1)
    {
        cout<<"Write value not loaded for slave"<<slave<<modbus_strerror(errno)<<endl;
    }
    else
    {
        cout<<"Write value is written"<<endl;
    }
}
