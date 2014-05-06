/* Minimum_Source*/

#include <Serial_OCM_APM.h>

#define TIMEOUT                 500
#define SEND_DATA_RATE          50
#define CALCULATE_BATTERY_RATE  1000


Dynamixel     Dxl(1);
word          drive;
word          steer;
byte_float    velocity; 
char          alarms[2];

word          last_drive;
word          last_steer;
short         last_position;
unsigned long absolut_last_time;
unsigned long blind_zone_entrance_time;
unsigned long timeout;
unsigned long battery_entrance_time;
unsigned long velocity_entrance_time;
unsigned long time;

boolean       blind_zone;
boolean       exit_edge;


void calculate_velocity(void);
void update_motors(void);

Serial_OCM_APM communication;
void setup() {
  // put your setup code here, to run once:
  
 Dxl.begin(1);
 Dxl.writeWord(1, 8, 0);
 last_position          = Dxl.readWord(1,36);
 absolut_last_time      = millis();
 velocity_entrance_time = millis();
 velocity.f             = 0.0f;
 blind_zone             = false;
 exit_edge              = false;
 drive                  = 0;
 steer                  = 0;
 last_drive             = 0;
 last_steer             = 0;

// Dxl.writeWord(1, 32, 1023);
}

void loop() {
  //SerialUSB.println(millis()-time);
  
  time = millis();
  
  
  communication.receive_message();
  if(time - velocity_entrance_time >= SEND_DATA_RATE){
//    char content[6];
//    byte_word angle;
//    
//    angle.w = Dxl.readByte( 1, 36);
//    calculate_velocity();
//    
//    content[0] = velocity.s[0];
//    content[1] = velocity.s[1];
//    content[2] = velocity.s[2];
//    content[3] = velocity.s[3];
//    
//    content[4] = angle.b[0];
//    content[5] = angle.b[1];
//    
//    //SerialUSB.println(content);
//    communication.send_message(GET_AB,content);
  }
  update_motors();
}

void calculate_velocity()
{
  short position     = Dxl.readWord(1,36);
  short step         = position-last_position;
  
  //time = millis();

  if(!blind_zone && (position == 1023 || position == 0 || (abs(step) > 150 && (position < 380 && position > 340))))
  {
    blind_zone    = true;
    blind_zone_entrance_time = time;
    timeout       = abs((293.0f/velocity.f)*204.6);
    exit_edge     = true;
  }
  else if(blind_zone  && position != 1023 && position != 0  && ( position < 100 || position > 923) && abs(step) < 150) blind_zone = false;
  
  switch(blind_zone)
  {
    case true:
    {  
      float aux_time = (293/velocity.f)*204.6;
      if((time - blind_zone_entrance_time) > timeout + TIMEOUT) velocity.f = 0.0f; 
      else if((time - blind_zone_entrance_time) > aux_time &&  aux_time > 0 ) velocity.f = 293*204.6/float(time-blind_zone_entrance_time);
      else if((time - blind_zone_entrance_time) > abs(aux_time))              velocity.f = -1*293*204.6/float(time-blind_zone_entrance_time);
     
      break;
    }
    
    case false:
    
      if(!exit_edge) velocity.f  = 293.0f*(float(step)/float(time-velocity_entrance_time)); //293 factor byte/millis to degrees/sec
      else           exit_edge = false;
      break;
  }
  velocity_entrance_time = time;
  last_position = position;
}

void update_motors()
{
  if(last_drive != drive)
  {
    Dxl.writeWord(1,32,drive);
    last_drive = drive;
  }
    if(last_steer != steer)
  {
    Dxl.writeWord(1,30,steer);
    last_steer = steer;
  }
}
