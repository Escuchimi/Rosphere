/* Minimum_Source*/
#define TIMEOUT                 500
#define CALCULATE_VELOCITY_RATE 50
#define CALCULATE_BATTERY_RATE  1000


Dynamixel     Dxl(1);
short         last_position;
unsigned long absolut_last_time;
unsigned long blind_zone_entrance_time;
unsigned long timeout;
unsigned long battery_entrance_time;
unsigned long velocity_entrance_time;
unsigned long time;
float         velocity; 
boolean       blind_zone;
boolean       exit_edge;


void calculate_velocity(void);

void setup() {
  // put your setup code here, to run once:
  
 Dxl.begin(1);
 Dxl.writeWord(1, 8, 0);
 last_position          = Dxl.readWord(1,36);
 absolut_last_time      = millis();
 velocity_entrance_time = millis();
 velocity               = 0.0f;
 blind_zone             = false;
 exit_edge              = false;
 Dxl.writeWord(1, 32, 1023);
}

void loop() {
  SerialUSB.println(millis()-time);
  
  time = millis();
  if(Serial3.available() 3
  if(time - velocity_entrance_time >= CALCULATE_VELOCITY_RATE) calculate_velocity(); 

  
  
 
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
    timeout       = abs((293.0f/velocity)*204.6);
    exit_edge     = true;
  }
  else if(blind_zone  && position != 1023 && position != 0  && ( position < 100 || position > 923) && abs(step) < 150) blind_zone = false;
  
  switch(blind_zone)
  {
    case true:
    {  float aux_time = (293/velocity)*204.6;
      if((time - blind_zone_entrance_time) > timeout + TIMEOUT) velocity = 0.0f; 
      else if((time - blind_zone_entrance_time) > aux_time &&  aux_time > 0 ) velocity = 293*204.6/float(time-blind_zone_entrance_time);
      else if((time - blind_zone_entrance_time) > abs(aux_time))              velocity = -1*293*204.6/float(time-blind_zone_entrance_time);
     break;
    }
    
    case false:
    
      if(!exit_edge) velocity  = 293.0f*(float(step)/float(time-velocity_entrance_time)); //293 factor byte/millis to degrees/sec
      else           exit_edge = false;
      break;
  }
  velocity_entrance_time = time;
  last_position = position;
}
