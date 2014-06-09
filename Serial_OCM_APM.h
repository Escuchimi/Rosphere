
#ifndef Serial_OCM_APM
#define Serial_OCM_APM

#include <Arduino.h>
//enum Board { APM, OCM};

#define BAUDRATE 115200
#define ACK_WAIT     10

enum msg_type {
    GET_AB            = 170,
    SET_AB            =  85,
    SHUTDOWN          = 219,
    E_SHUTDOWN        = 109,
    SETUP             = 182,
    ACKNOWLEDGE       =  36,
    ALARM             = 254,
};

typedef union byte_float{
        float f;
        char s[sizeof float];
}byte_float;

typedef struct Acknowledge{
		bool          state;
		unsigned long time;
		msg_type      type;
}Acknowledge;
                           
class Serial_OCM_APM{
    public:
        void Serial_OCM_APM(Board);
        void ~Serial_OCM_APM(void);
    
        void send_message(msg_type, int);
        void receive_message(void);
    
        void Ack_checker(void);
                               
    private:
        //void uart_print(char);
    
        Acknowledge Ack_vec[3];
                               
    }

#endif
