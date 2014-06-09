#include <Serial_OCM_APM.h>


#define TIMEOUT                 500
#define SEND_DATA_RATE          50
#define SEND_TEMP_BATTERY_RATE  60000
#define LOWEST_VOLTAGE_ALLOWED  100   //in 0.1v units
#define HIGHEST_VOLTAGE_ALLOWED 140   //
#define REFRESH_UPDATE_MOTORS   1000

/*     Variables globales de la controladora */
Dynamixel     Dxl(1);             // DaisyChain de motores conectados a USART '1'
word          drive;              // Valor de input del motor de giro continuo (en bytes de % de par maximo)
word          steer;              // Valor de input del motor de giro lateral  (En bytes de angulo de inclinacion)
byte_float    velocity;           // Valor medido/calculado de la velocidad del motor de giro continuo
boolean       emergency_shutdown; // Flag for shutting down the power of the motors i case of an emergency


/*    Variables internas necesitadas para hacer calculos */
word          last_drive;                 // Ultimo valor de input del motor de giro continuo, para comprobar flanco
word          last_steer;                 // Ultimo valor de input del motor de giro lateral, para comprobar flanco
short         last_position;              // Ultima posicion leida por el motor de giro continuo 
unsigned long blind_zone_entrance_time;   // Tiempo de entrada en la zona ciega por el motor de giro continuo (GC)
unsigned long timeout;                    // Tiempo maximo estimado de salida de la zona ciega por el motor de GC
unsigned long datasend_entrance_time;     // Tiempo del ultimo envio de la carga de la bateria y la temperatura
unsigned long velocity_entrance_time;     // Tiempo de ultimo calculo de velocidad
unsigned long time;                       // Tiempo en el que se empezo a ejecutar el loop()
unsigned long last_time_updated;

boolean       blind_zone;                 // Estado de el motor de GC, '0' -> Fuera de la zona muerta; '1' -> Dentro de la zona muerta
boolean       exit_edge;                  // Flanco de salida de la zona muerta para evitar transitorios debidos al cambio

float         acceleration;


/*     Funciones utilizadas en el bucle principal */
void calculate_velocity(void); // Funcion que calcula la velocidad del motor de giro continuo
void update_motors(void);      // Funcion que actualiza el valor de input de los motores si se ha recibido un cambio en la señal de consigna
void update_send_data(void);

/* Declaracion de una variable Serial_OCM_APM para la comunicacion con la Ardupilot */
Serial_OCM_APM communication; 


void setup() {
  
 /* Inicializacion de todas las variables */
   /* Setup de los motores */
 Dxl.begin(1);                                 // Mbps de comunicacion con los motores
 Dxl.writeWord(1, 8, 0);                       // Configuracion del motor '1' como motor de giro continuo 
 Dxl.writeWord(2, 6, 0);
 Dxl.writeWord(2, 8, 1023);
 Dxl.writeByte(254, 12, LOWEST_VOLTAGE_ALLOWED);
 Dxl.writeByte(254, 13, HIGHEST_VOLTAGE_ALLOWED);
 
 //Dxl.writeWord(1,32,512);
 
 
 
 last_position             = Dxl.readWord(1,36);  
 velocity_entrance_time    = millis();
 datasend_entrance_time    = millis()+25;    //in order to make both sending functions be in different phase and make the mmicroprocessor use more homogeneous
 last_time_updated         = millis();
 velocity.f                = 0.0f;
 acceleration              = 0.0f;
 blind_zone                = false;
 exit_edge                 = false;
 drive                     = 0;
 steer                     = 512;
 last_drive                = 0;
 last_steer                = 0;
}

void loop() {
  
    time = millis();
  
   
    communication.receive_message();  // Recepcion de ordenes de la APM
  
    update_send_data();               // Sends data if needed
      
    update_motors();                  // Actualizacion del input de los motores
}


/* Function that checks if data must be sent to the Ardupilot Mega
  All the data is sended periodically defined by time constants:
      -ANGULAR velocity of the Rosphere drive motor and angle of the steer motor. Rate defined by SEND_DATA_RATE
      -Current voltage of the batteries and temperature of both motors. Rate defined by SEND_TEMP_BATTERY_RATE
*/
void update_send_data()
{
    /*    Envio de la velocidad a la APM cada 50 ms */
    if(time - velocity_entrance_time >= SEND_DATA_RATE)
    {
          
          char  content[6];                             // Contenido a enviar
          byte_word angle;
    
          angle.w = Dxl.readWord(2, 36);                // Medida del angulo actual del motor de GL
          calculate_velocity();                         // Calculo de la velocidad del motor de GC llamando a la funcion correspondiente
          
          
        /*   Prepararacion y empaquetamiento de la variable content para enviarla a la APM */
          content[0] = velocity.s[0];
          content[1] = velocity.s[1];
          content[2] = velocity.s[2];
          content[3] = velocity.s[3];
    
          content[4] = angle.b[0];
          content[5] = angle.b[1];
                    
        /*   -------------------------------------   */
        
          communication.send_message(GET_AB,content);  // Envio de la informacion 'content' como tipo de informacion 'GET_AB' a la APM
    }
  /*  -------------------------------------------------  */
  
    if(time - datasend_entrance_time >= SEND_TEMP_BATTERY_RATE)
    {
      byte_float drive_temperature;
      byte_float steer_temperature;
      byte_float voltage;
      
      char content[12];
      
      drive_temperature.f = (float) Dxl.readByte(1,43);
      steer_temperature.f = (float) Dxl.readByte(2,43);
      
      voltage.f = (float)Dxl.readByte(2,42)/10;
      
      content[0]  = drive_temperature.s[0];
      content[1]  = drive_temperature.s[1];
      content[2]  = drive_temperature.s[2];
      content[3]  = drive_temperature.s[3];
      
      content[4]  = steer_temperature.s[0];
      content[5]  = steer_temperature.s[1];
      content[6]  = steer_temperature.s[2];
      content[7]  = steer_temperature.s[3];
      
      content[8]  = voltage.s[0];
      content[9]  = voltage.s[1];
      content[10] = voltage.s[2];
      content[11] = voltage.s[3];
      
      communication.send_message(GENERAL_DATA,content);
      datasend_entrance_time = time;
    }
}

/* Function for updating the input value that the motors must have
   if the value is the same that the last one, it doesn't do nothing, for not overloading the motors
   (the communication with the motors takes a while)peor 
*/
void update_motors()
{
  if(last_steer != steer)
  {
    Dxl.writeWord(2,30,steer);
    last_steer = steer;
    
    //SerialUSB.println(steer);
    last_time_updated = time;
  }
  if(last_drive != drive)
  {
    Dxl.writeWord(1,32,drive);
    last_drive = drive;
    
    last_time_updated = time;
  }
 
  if(time - last_time_updated >= REFRESH_UPDATE_MOTORS)
  {
     Dxl.writeWord(2,30,steer);
    last_steer = steer;

    Dxl.writeWord(1,32,drive);
    last_drive = drive;
    
    last_time_updated = time;
  }
}

/* Function for calculating velocity of the drive motor in grad/s.
    -When it enters the blind zone, it keeps the last velocity read before entering as the current velocity
    -If it doesn't exit the blind zone after the time estimated that would take to exit it within the said velocity plus a TIMEOUT, the velocity estimated starts to decrease until its 0
    -The first velocity measured after exiting the blind zone stills being the estimate due to posible discontinuities
    -The rest of the velocities are calculated by
                                                    current_position-last_position
                                                   ---------------------------------
                                                        Time spent between both
*/
void calculate_velocity()
{
  short position              = Dxl.readWord(1,36);
  short step                  = position-last_position;
  float last_velocity         = velocity.f;
  unsigned long sampling_time = time - velocity_entrance_time;
  
  // Seeing if we are inside the blind zone or not and marking it
  if(!blind_zone && (position == 1023 || position == 0 || (abs(step) > 150 && (position < 380 && position > 340))))
  {
    blind_zone    = true;
    blind_zone_entrance_time = time;
    timeout       = abs((293.0f/velocity.f)*204.8);      //204.8 bytes corresponding to 60 degrees
    exit_edge     = true;
  }
  else if(blind_zone  && position != 1023 && position != 0  && ( position < 100 || position > 923) && abs(step) < 150) blind_zone = false;
  // -------------------------------------------------------------
  
  
  //Calculate de velocity either if we are in the blind zone or not
  switch(blind_zone)
  {
    case true:
    {  
      float aux_time = (293/velocity.f)*204.8;
      
      if((time - blind_zone_entrance_time) > timeout + TIMEOUT) velocity.f = 0.0f; 
      else if((time - blind_zone_entrance_time) > aux_time &&  aux_time > 0 ) velocity.f = 293*204.8/float(time-blind_zone_entrance_time);
      else if((time - blind_zone_entrance_time) > abs(aux_time))              velocity.f = -1*293*204.8/float(time-blind_zone_entrance_time);
     
      break;
    }
    
    case false:
    {
      if(!exit_edge) velocity.f  = 293.0f*(float(step)/float(sampling_time)); //293 factor byte/millis to degrees/sec
      else           exit_edge = false;
      
      break;
    }
  }
  // ----------------------------------------------
  
  acceleration = float(velocity.f-last_velocity)/float(sampling_time);
  velocity_entrance_time = time;
  last_position = position;
  //SerialUSB.println(acceleration);
}


