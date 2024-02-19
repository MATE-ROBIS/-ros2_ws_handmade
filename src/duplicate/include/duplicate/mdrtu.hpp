#include<modbus/modbus-rtu.h>

#include<iostream>

using namespace std;

class Mdrtu{
    public:
        Mdrtu(char* port,int baud_rate,char parity,int data_bit,int stop_bit):port_ {port},baud_rate_ {baud_rate},data_bit_{data_bit},stop_bit_{stop_bit}{};
 
    bool modbus_connection();
   
        
    private:
        modbus_t* ctx;
        char* port_ ;
        int baud_rate_;
        char parity_;
        int data_bit_;
        int stop_bit_;

};

bool Mdrtu::modbus_connection(){

    ctx=modbus_new_rtu(port_,baud_rate_,parity_,data_bit_,stop_bit_);

    if(ctx==nullptr){
        cerr<<"Context is not created! Error_id"<<modbus_strerror(errno)<<endl;
        return false;
        modbus_connect(ctx);

    }else{
        modbus_set_slave(ctx,1);
        modbus_set_slave(ctx,2);
        cout<<"connected"<<endl;
        

    }


}

