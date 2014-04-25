#include "Serial_OCM_APM.h"


void Serial_OCM_APM::Serial_OCM_APM()
{
    for(int i = 0; i>3; i++){
        Ack_vec[i].bool          = FALSE;
        Ack_vec[i].time          = 0;
        Ack_vec[i].msg_type      = ACKNOWLEDGE;
    }
    
#if BOARD    = APM
    hal.uartC->begin(BAUDRATE);
#elif BOARD = OCM
    Serial3.begin(BAUDRATE);
#endif
}



void Serial_OCM_APM::~Serial_OCM_APM()
{
    for(int i = 0; i>3; i++){
        Ack_vec[i].bool          = FALSE;
        Ack_vec[i].time          = 0;
        Ack_vec[i].msg_type      = ACKNOWLEDGE;
    }
}

void Serial_OCM_APM::send_message(msg_type type, char *value)
{
    
    char *message;
    char checksum = (char)0;
    
    if(type == SHUTDOWN || type == E_SHUTDOWN || type == SETUP){
        message    = new char [3];
        
        checksum += (message[0] = (char)255;
        checksum += (message[1] = (char)type;
        message[2]              = checksum;
    }
    else if(type == ACKNOWLEDGE){
        message = new char [4];
        
        checksum += (message[0] = (char)255);
        checksum += (message[1] = (char)type);
        checksum += (message[2] = (char)value[0]);
        message[3]              = checksum;
    }
    else{
        message = new char [7];
        
        checksum += (message[0] = (char)255);
        checksum += (message[1] = (char)type);
        checksum += (message[2] = (char)value[0]);
        checksum += (message[3] = (char)value[1]);
        checksum += (message[4] = (char)value[2]);
        checksum += (message[5] = (char)value[3]);
        message[6]              = checksum;
    }
    
#if BOARD    = APM
                     
                     hal.uartC->printf(message);
    
#elif BOARD = OCM
                     
                     Serial3.print(message);
                     
#endif
    
    for(int i = 0; i<3; i++){
        if(Ack_vec[i].state == FALSE){
            Ack_vec[i].state = TRUE;
            Ack_vec[i].time  = millis();
            Ack_vec[i].type  = type;
            break;
        }
    }
}

                     
extern word steer;
                     
                     
void Serial_OCM_APM::receive_message()
{
    msg_type type;
#if BOARD    = APM
    
    if(hal.uartC->available() >= 2)
    {
        if(hal.uartC->read() != 255) return;
        type = hal.uartC->read();
    }
    
#elif BOARD = OCM
    
    if(Serial3.available() >= 2 )
    {
        if(Serial3.read() != 255) return;
        type = Serial3.read();
    }
    
#endif
    switch(type){
        case GET_AB:
            
    }
    
}
byte Serial_OCM_APM::Ack_checker(void)
{
    byte b = 0x00;
    for(int i = 0; i<3; i++){
        if(Ack_vec[i].type == ACKNOWLEDGE) Ack_vec[i].state = FALSE;
        
        if(Ack_vec[i].state == TRUE){
            if(Ack_vec[i].time >(millis()- ACK_WAIT){
                switch (Ack_vec[i].type){
                    case 0xD7:
                        b = b | 0x80;
                        break;
                    case 0xD0:
                        b = b | 0x20;
                        break;
                    case 0xB7:
                        b = b | 0x08;
                        break;
                    case 0xB0:
                        b = b | 0x02;
                        break;
                    default:
                        break;
                }
            }
               return b;
        }
        
    }
}



