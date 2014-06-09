#include "Serial_OCM_APM.h"


void Serial_OCM_APM::Serial_OCM_APM()
{
    for(int i = 0; i>3; i++){
        Ack_vec[i].state          = FALSE;
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
        Ack_vec[i].state          = FALSE;
        Ack_vec[i].time          = 0;
        Ack_vec[i].msg_type      = ACKNOWLEDGE;
    }
}

void Serial_OCM_APM::send_message(msg_type type, char *value)
{
    
    char *message;
    char checksum = (char)0;
    
    switch(type){
            
        case ACKNOWLEDGE:
            message = new char [5];
        
            checksum += (message[0] = (char)255);
            checksum += (message[1] = (char)type);
            checksum += (message[2] = (char)value[0]);
            message[3]              = checksum;
            message[4]              = '\0';
            
            break;
            
        case SET_AB:
            message = new char [8];
        
            checksum += (message[0] = (char)255);
            checksum += (message[1] = (char)type);
            checksum += (message[2] = (char)value[0]);
            checksum += (message[3] = (char)value[1]);
            checksum += (message[4] = (char)value[2]);
            checksum += (message[5] = (char)value[3]);
            message[6]              = checksum;
            message[7]              = '\0';
            
            break;
                     
        case GET_AB:
            message = new char [10];
        
            checksum += (message[0] = (char)255);
            checksum += (message[1] = (char)type);
            checksum += (message[2] = (char)value[0]);
            checksum += (message[3] = (char)value[1]);
            checksum += (message[4] = (char)value[2]);
            checksum += (message[5] = (char)value[3]);
            checksum += (message[6] = (char)value[4]);
            checksum += (message[7] = (char)value[5]);
            message[8]              = checksum;
            message[9]              = '\0';
                     
            break;
            
        case ALARM:
            message = new char [6];
            
            checksum += (message[0] = (char)255);
            checksum += (message[1] = (char)type);
            checksum += (message[2] = (char)value[0]);
            checksum += (message[3] = (char)value[1]);

            message[4]              = checksum;
            message[5]              = '\0';
            
            break;
            
        case default:
                     
            message    = new char [4];
                     
            checksum += (message[0] = (char)255;
            checksum += (message[1] = (char)type;
            message[2]              = checksum;
            message[3]              = '\0';
            
            break;
    }

#if BOARD    = APM
                     
                     hal.uartC->printf(message);
    
#elif BOARD = OCM
                     
                     Serial3.print(message);
                     
#endif
    
    delete[] message;
    
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
extern word drive;
extern byte alarms[2];

extern float velocity;
extern word  angle;
                     
void Serial_OCM_APM::receive_message()
{
    msg_type type;
    byte     checksum;
#if BOARD   == APM
    
    if(hal.uartC->available() >= 2)
    {
        if(hal.uartC->read() != 255) return;
        type = hal.uartC->read();
    }
    
#elif BOARD == OCM
    
    if(Serial3.available() >= 2 )
    {
        if(Serial3.read() != 255) return;
        type = Serial3.read();
    }
    
#endif
    switch(type){
        case SET_AB:
        {
            byte lowbyte_drive,highbyte_drive,lowbyte_steer,highbyte_steer;
#if BOARD == OCM
            highbyte_drive = Serial3.read();
            lowbyte_drive  = Serial3.read();
            
            highbyte_steer = Serial3.read();
            lowbyte_steer  = Serial3.read();
            
            checksum = 255 + type + lowbyte_drive + highbyte_drive + lowbyte_steer + highbyte_steer;
            if(checksum != Serial3.read())return;
#endif
            
            drive = word(highbyte_drive,lowbyte_drive);
            steer = word(highbyte_steer,lowbyte_steer);
            break;
        }
        case GET_AB:
        {
            byte lowbyte_angle,highbyte_angle;
            byte_float vel;
#if BOARD == APM
            vel.s[0] = hal.uartC->read();
            vel.s[1] = hal.uartC->read();
            vel.s[2] = hal.uartC->read();
            vel.s[3] = hal.uartC->read();
            
            highbyte_angle = hal.uartC->read();
            lowbyte_angle  = hal.uartC->read();
            
            checksum = 255 + type + vel.s[0] + vel.s[1] + vel.s[2] + vel.s[3]
            if(checksum != hal.uartC->read())return;
            
            angle    = word(highbyte_angle,lowbyte_angle);
            velocity = vel.f;
#endif
        }
        case ACKNOWLEDGE:
        {
            byte ack;
#if BOARD == OCM
            ack = Serial3.read();
            
            checksum = 255 + type + ack;
            if(checksum != Serial3.read())return;
#endif
            
            for(int i = 0; i>3; i++)
                if(Ack_vec[i].state == true && ack == Ack_vec[i].msg_type)Ack_vec[i].state = false;
            
            break;
        }
        case ALARM:
        {
            byte alarm_drive, alarm_steer;
#if BOARD == APM
            alarm_drive = hal.uartC->read();
            alarm_steer = hal.uartC->read();
            
            checksum = 255 + type + alarm_drive + alarm_steer;
            if(checksum != hal.uartC->read())return;
            
            alarms[0] = alarm_drive;
            alarms[1] = alarm_steer;
            
            break;
        }
        case default:
            //HACE FALTA RELLENAR CON EL RESTO DE CASOS
            break;
    }
    
}
                         
// REVISAR los .type
byte Serial_OCM_APM::Ack_checker(void)
{
    byte b = 0x00;
    for(int i = 0; i<3; i++){
        if(Ack_vec[i].type == ACKNOWLEDGE || Ack_vec[i].type == GET_AB) Ack_vec[i].state = FALSE;
        
        if(Ack_vec[i].state == TRUE){
            if(Ack_vec[i].time >(millis()- ACK_WAIT){
                switch (Ack_vec[i].type){
                    case 85:
                        b = b | 0x80;
//                        break;
//                    case 0xD0:
//                        b = b | 0x20;
//                        break;
//                    case 0xB7:
//                        b = b | 0x08;
//                        break;
//                    case 0xB0:
//                        b = b | 0x02;
//                        break;
                    default:
                        break;
                }
            }
               return b;
        }
        
    }
}



