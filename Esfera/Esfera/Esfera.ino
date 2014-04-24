/* Minimum_Source*/
#define TIMEOUT 500

Dynamixel     Dxl(1);
short         last_position;
unsigned long last_time;
unsigned long entrance_time;
unsigned long timeout;
float         velocity; 
boolean       blind_zone;
boolean       exit_edge;


void calculate_velocity(void);

void setup() {
  // put your setup code here, to run once:
  
 Dxl.begin(1);
 Dxl.writeWord(1, 8, 0);
 last_position = Dxl.readWord(1,36);
 last_time     = millis();
 velocity      = 0.0f;
 blind_zone    = false;
 exit_edge     = false;
 Dxl.writeWord(1, 32, 1200);
}

void loop() {

 
  calculate_velocity();
  SerialUSB.println(velocity);
  delay(50);
  
  //SerialUSB.println(Dxl.readWord(1,38));
  //if(Dxl.readWord(1,36) == 1023) SerialUSB.print("EYY");
  // put your main code here, to run repeatedly: 
}

void calculate_velocity()
{
  short position = Dxl.readWord(1,36);
  short step;
  unsigned long time = millis();
  step = position-last_position;
  if(!blind_zone && (position == 1023 || position == 0 || (abs(step) > 150 && (position < 380 && position > 340))))
  {
    blind_zone    = true;
    entrance_time = time;
    timeout       = abs((293.0f/velocity)*204.6);
    exit_edge     = true;
  }
  else if(blind_zone  && position != 1023 && position != 0  && ( position < 100 || position > 923) && abs(step) < 150) blind_zone = false;
  
  switch(blind_zone)
  {
    case true:
    {  float aux_time = (293/velocity)*204.6;
      if((time - entrance_time) > timeout + TIMEOUT) velocity = 0.0f; 
      else if((time - entrance_time) > aux_time &&  aux_time > 0 ) velocity = 293*204.6/float(time-entrance_time);
      else if((time - entrance_time) > abs(aux_time))            {  velocity = -1*293*204.6/float(time-entrance_time);SerialUSB.print(time-entrance_time);}
     break;
    }
    
    case false:
    
      if(!exit_edge) velocity  = 293.0f*(float(step)/float(time-last_time)); //293 factor byte/millis to degrees/sec
      else           exit_edge = false;
      break;
  }
//  SerialUSB.print("Step = ");
//  SerialUSB.print(step);
//  SerialUSB.print("  Time increment = ");
//  SerialUSB.print(time-last_time);
//  SerialUSB.print("
  SerialUSB.print(blind_zone);
  SerialUSB.print(" - ");
 // SerialUSB.println(position);
  last_time = time;
  last_position = position;
}
