#include <AP_HAL.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_InertialSensor.h>
#include <AP_ADC.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>            // ArduPilot Mega Barometer Library
#include <AP_GPS.h>
#include <AP_AHRS.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <AP_Baro.h>
#include <GCS_MAVLink.h>
#include <Filter.h>
#include <SITL.h>
#include <AP_Buffer.h>
#include <AP_Notify.h>
#include <AP_Vehicle.h>
#include <DataFlash.h>

#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

#include <DataTypes.h>

#include <Serial_OCM_APM.h>
#include <Serial_APM_RPI.h>

// INS and Baro declaration
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
AP_InertialSensor_MPU6000 ins;
AP_Baro_MS5611 baro(&AP_Baro_MS5611::spi);
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
AP_ADC_ADS7844 adc;
AP_InertialSensor_Oilpan ins( &adc );
AP_Baro_BMP085 baro;
#else
AP_InertialSensor_HIL ins;
#endif

AP_Compass_HMC5843 compass;
#define T6 1000000
#define T7 10000000

//AP_GPS_406 *gps;
GPS *g_gps;

AP_GPS_Auto g_gps_driver(&g_gps);

// choose which AHRS system to use
AP_AHRS_DCM  ahrs(ins, baro, g_gps);

AP_Baro_HIL barometer;


#define HIGH 1
#define LOW 0

#define SEND_MOVEMENT_RATE        50
#define SEND_IMU_RATE             50
#define REFRESH_UPDATE_CONTROLLER 1000
#define CALCULATE_REGULATOR_RATE  50
//#define CONVERSION                0.29296875

Serial_OCM_APM communication_OCM;
Serial_APM_RPI communication_RPI;

void update_motor_controler(void);
void regulator(void);


//Real values of variables, measured in the motor controller or with the IMU 
float          velocity_real;
float          roll_real;
float          heading_real;
word           angle;
float          roll;
float          pitch;
float          yaw;

float          temperature_drive;
float          temperature_steer;
float          voltage;

//Desired values, setpoints of the regulators, modified by the model and/or (depends if it's teleoperated) directly from the Raspberry Pi commands
float          velocity_desired;
float          roll_desired;
float          heading_desired;
float          drive_torque;


unsigned long  time;   

// Internal variables needed in order to make the program work
unsigned long movement_entrance_time;
unsigned long imu_entrance_time;
unsigned long last_time_updated;
unsigned long last_time_regulated;
float         last_drive_torque;
float         last_roll_desired;

void calculate_inclinations(void);
void update_send_data(void);


void setup(void)
{

#ifdef APM2_HARDWARE
  // we need to stop the barometer from holding the SPI bus
  hal.gpio->pinMode(40, GPIO_OUTPUT);
  hal.gpio->write(40, HIGH);
#endif

  ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ);

  ins.init_accel();

  ahrs.init();

  if( compass.init() )ahrs.set_compass(&compass);

//gps->init(hal.uartB);

  g_gps = &g_gps_driver;
#if WITH_GPS
  g_gps->init(hal.uartB);
#endif

  velocity_real          = 0.0;
  roll_real              = 0.0;
  heading_real           = 0.0;

  velocity_desired       = 0.0;
  roll_desired           = 0.0;
  heading_desired        = 0.0;


  movement_entrance_time = hal.scheduler->millis();
  imu_entrance_time      =  hal.scheduler->millis() + 25;
  last_time_updated      =  hal.scheduler->millis();
  

  hal.console->begin(115200);
}


void loop(void)
{
  time = hal.scheduler->millis();

  calculate_inclinations();

  update_send_data();

  communication_OCM.receive_message();
  communication_RPI.receive_message();
  
  //-------------------------------------------------------------------
  drive_torque = velocity_desired;
  roll_desired = heading_desired;
  //-------------------------------------------------------------------

//  regulator();
  update_motor_controller();
}

AP_HAL_MAIN();

void update_motor_controller()
{
  if(last_drive_torque != drive_torque || last_roll_desired != roll_desired)
  {
    char content[4];

    byte_word drive;
    byte_word steer;

    if      (drive_torque >  5.0) drive_torque = 5.0;
    else if (drive_torque < -5.0) drive_torque = -5.0;
    
    if      (drive_torque <  0.0) drive.w = mapfloat(drive_torque, 0.0, -5.0, 0, 1023);
    else if (drive_torque == 0.0) drive.w = 1024;
    else if (drive_torque >  0.0) drive.w = mapfloat(drive_torque, 0.0, 5.0, 1025, 2047);

    steer.w = mapfloat(roll_desired, -45.0, 45.0, 0, 1023);

    content[0] = drive.b[0];
    content[1] = drive.b[1];
    content[2] = steer.b[0];
    content[3] = steer.b[1];

    communication_OCM.send_message(SET_AB,content);

    last_drive_torque = drive_torque;
    last_roll_desired = roll_desired;
    
    last_time_updated = time;
  }
  else if(time - last_time_updated >= REFRESH_UPDATE_CONTROLLER)
  {
    char content[4];

    byte_word drive;
    byte_word steer;
    
    if      (drive_torque <  0.0) drive.w = mapfloat(drive_torque, 0.0, -5.0, 0, 1023);
    else if (drive_torque == 0.0) drive.w = 1024;
    else if (drive_torque >  0.0) drive.w = mapfloat(drive_torque, 0.0, 5.0, 1025, 2047);

    steer.w = mapfloat(roll_desired, -45.0, 45.0, 0, 1024);

    content[0] = drive.b[0];
    content[1] = drive.b[1];
    content[2] = steer.b[0];
    content[3] = steer.b[1];

    communication_OCM.send_message(SET_AB,content);
    
    last_time_updated = time;
  }
}

void update_send_data()
{
  if(time - movement_entrance_time >= SEND_MOVEMENT_RATE)
  {
    char       content[28];

    byte_float vel;
    byte_float head;
    byte_float sending_roll;
    byte_float sending_pitch;
    byte_float sending_yaw;
    byte_float ref_vel;
    byte_float ref_head;

    ref_vel.f       = velocity_desired; 
    ref_head.f      = heading_desired;
    vel.f           = velocity_real;
    head.f          = heading_real;
    sending_roll.f  = roll;
    sending_pitch.f = pitch;
    sending_yaw.f   = yaw;


    for(int i=0; i<4; i++)
    {
      content[i]    = ref_vel.s[i];
      content[i+4]  = ref_head.s[i];
      content[i+8]  = vel.s[i];
      content[i+12] = head.s[i];
      content[i+16] = sending_roll.s[i];
      content[i+20] = sending_pitch.s[i];
      content[i+24] = sending_yaw.s[i];
    }

    //communication_RPI.send_message(MOVEMENT_DATA,content);
    communication_RPI.send_message(IDENT_DATA,content);  

    movement_entrance_time = time;
  }
  
//  if(time - imu_entrance_time >= SEND_IMU_RATE)
//  {
//    char       content[12];
//    
//    byte_float sending_roll;
//    byte_float sending_pitch;
//    byte_float sending_yaw;
//
//    sending_roll.f  = roll;
//    sending_pitch.f = pitch;
//    sending_yaw.f   = yaw;
//    
//    for(int i = 0; i<4; i++)
//    {
//      content[i]   = sending_roll.s[i];
//      content[i+4] = sending_pitch.s[i];
//      content[i+8] = sending_yaw.s[i];
//    }
//    
//    communication_RPI.send_message(IMU,content);
//    
//    imu_entrance_time = time;
//  }
}

void regulator(void)
{
  if(time - last_time_regulated >= CALCULATE_REGULATOR_RATE)
  {
    drive_torque = velocity_desired;
    roll_desired = heading_desired-roll+mapfloat(angle,0,1024,45.0,-45.0);
    
    last_time_regulated = time;
  }
}



void calculate_inclinations(void)
{
  float         internal_heading = 0;
  if(compass.read())
  {
    internal_heading = compass.calculate_heading(ahrs.get_dcm_matrix());

    //g_gps->update();

  }

  ahrs.update();

  heading_real = compass.use_for_yaw() ? ToDeg(internal_heading) : 0.0;
//   if (gps->new_data) {
//        hal.console->print("gps:");
//        hal.console->print(" Lat:");
//        hal.console->print((float)gps->latitude / T7, BASE_DEC);
//        hal.console->print(" Lon:");
//        hal.console->print((float)gps->longitude / T7, BASE_DEC);
//        hal.console->print(" Alt:");
//        hal.console->print((float)gps->altitude_cm / 100.0, BASE_DEC);
//        hal.console->print(" GSP:");
//        hal.console->print(gps->ground_speed_cm / 100.0);
//        hal.console->print(" COG:");
//        hal.console->print(gps->ground_course_cd / 100, BASE_DEC);
//        hal.console->print(" SAT:");
//        hal.console->print(gps->num_sats, BASE_DEC);
//        hal.console->print(" FIX:");
//        hal.console->print(gps->fix, BASE_DEC);
//        hal.console->print(" WEEK:");
//        hal.console->print(gps->time_week, BASE_DEC);
//        hal.console->print(" TIM:");
//        hal.console->print(gps->time_week_ms, BASE_DEC);
//        hal.console->println();
//        gps->new_data = 0; // We have readed the data
//    }

// Different than the ardupilot ones due to the orientation of the ArduPilot Mega within the sphere
   pitch = ToDeg(ahrs.roll)*(-1);
   roll  = ToDeg(ahrs.pitch);
   yaw   = ToDeg(ahrs.yaw)+90.0;
//------------------------------------------
}

int mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (int)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}




