/*!
* @file globals.h
*
* It is a file to define all global variables
* Inspired heavily by SatNOGs Rotator controller but modified
* 
* Licensed under MIT
*/
#ifndef LIBRARIES_GLOBALS_H_
#define LIBRARIES_GLOBALS_H_

///Motor Configuration Struct
struct Motor_Configs{
  boolean type;         ///< True = Az, False = El
  uint16_t nativeSPR;   ///< Steps Per Revolution of stepper motor, native to motor
  uint16_t microstep;   ///< microstep setting
  uint16_t SPR;         ///< Steps per Rev of motor, nativeSPR * microstep
  float minStepPeriod;  ///< Default step period in microseconds
  float minPulseWidth;  ///< Minimum step period in microseconds
  float gRatio;         ///< gear ratio of axis
  float MilliAmps;      ///< Max Current Draw of Motor
  float minPos;         ///< Minimum Position of Axis, in degrees
  float maxPos;         ///< Maximum Position of Axis, in degrees
  float maxSpeed;       ///< Max Speed, steps/s, consider microstep
  float maxAccel;       ///< Max Acceleration, steps/s^2, consider microstep
  float homeSpeed;      ///< Speed during homing process
  uint32_t homeStep;    ///< homing step margin, in motor steps (not degrees)
  uint32_t homeDelay;   ///< Homing delay in milliseconds for the axis
};

Motor_Configs el_cfg = {
  .type = false,        ///< True = Az, False = El
  .nativeSPR = 200,     ///< Steps Per Revolution of stepper motor, native to motor
  .microstep = 8,       ///< Microstep Mode of Motor, 
  .SPR = 1600,          ///< nativeSPR * microstep       
  .minStepPeriod = 100, ///< Step Period in microseconds
  .minPulseWidth = 20,  ///< Minimum Pulse Width in microseconds
  .gRatio = 8.0,        ///< Gearing Ratio of Motor to Dish
  .MilliAmps = 1500,     ///< Max Current Draw of Motor
  .minPos = 21.0,       ///< Minimum Position, in degrees
  .maxPos = 95.0,       ///< Maximum Position, in degrees
  .maxSpeed = 3200,     ///< Max Speed, steps/s, consider microstep
  .maxAccel = 1600,     ///< Max Acceleration, steps/s^2, consider microstep
  .homeSpeed = 500,     ///< Speed during homing process
  .homeStep = 50,       ///< homing step margin, in motor steps (not degrees)
  .homeDelay = 15000    ///< Homing delay in milliseconds for the axis
};

Motor_Configs az_cfg = {
  .type = true,         ///< True = Az, False = El
  .nativeSPR = 200,     ///< Steps Per Revolution of stepper motor, native to motor
  .microstep = 8,       ///< Microstep Mode of Motor
  .SPR = 1600,          ///< nativeSPR * microstep
  .minStepPeriod = 100, ///< Step Period in microseconds
  .minPulseWidth = 20,  ///< Minimum Pulse Width in microseconds
  .gRatio = 12.6,       ///< Gearing Ratio of Motor to Dish
  .MilliAmps = 1500,    ///< Max Current Draw of Motor
  .minPos = 0.0,        ///< Minimum Position, in degrees
  .maxPos = 360.0,      ///< Maximum Position, in degrees
  .maxSpeed = 3200,     ///< Max Speed, steps/s, consider microstep
  .maxAccel = 1600,     ///< Max Acceleration, steps/s^2, consider microstep
  .homeSpeed = 1000,     ///< Speed during homing process
  .homeStep = 50,       ///< homing step margin, in motor steps (not degrees)
  .homeDelay = 15000    ///< Homing delay in milliseconds for the axis
};

struct Motor_Pins{
  uint8_t dir;          ///< Direction Control Pin
  uint8_t step;         ///< Step Pin
  uint8_t nSleep;       ///< Sleep Pin, Active LOW
  uint8_t enable;       ///< Enable Pin, Active HIGH
  uint8_t nFault;       ///< Fault Pin, Active LOW
  uint8_t chipSelect;   ///< SPI Chip Select Pin
};

enum _control_mode {
    position = 0, speed = 1
};
/** Rotator status */
enum _rotator_state {
    /*
    boot     - initial bootup of microcontroller
    disabled - motor driver disabled, timers disabled, calibration lost
    idle     - motor driver enabled, timers disabled.
    homing   - calibration of az/el position at boot
    active   - motors moving to target
    error    - motor in error state....do something to revcover
    */
    boot = 0, disabled = 1, homing = 2, idle = 4, active = 8, error = 16
};
/** Rotator Errors */
enum _rotator_error {
    no_error = 1, sensor_error = 2, homing_error = 4, motor_error = 8,
    over_temperature = 12, wdt_error = 16
};

struct _rotator{
    volatile enum _rotator_state rotator_state;   ///< Rotator status
    volatile enum _rotator_error rotator_error;   ///< Rotator error
    enum _control_mode control_mode;              ///< Control mode
    bool homing_flag;                             ///< Homing flag
    bool calibrated;                              ///< True if both motors are calibrated
    int8_t inside_temperature;                    ///< Inside Temperature
    double park_az, park_el;                      ///< Park position for both axis
    volatile uint8_t el_fault, az_fault;          ///< Motor drivers fault flag
};

_rotator rotator = { .rotator_state = boot, .rotator_error = no_error,
                     .control_mode = position, .homing_flag = false,
                     .calibrated = false,
                     .inside_temperature = 0, .park_az = 0, .park_el = 0,
                     .el_fault = LOW, .az_fault = LOW};


//Control inputs accessed from both main loop and Comm protocol.
/// These variables are in degrees
struct _control{
    double input;          ///< Motor Position feedback in deg
    double input_prv;      ///< T-1 Motor Position feedback in deg
    double speed;          ///< Motor Rotation speed in deg/s
    double setpoint;       ///< Position set point in deg
    double setpoint_speed; ///< Speed set point in deg/s
    uint16_t load;         ///< Motor Load in mA
    double u;              ///< Control signal range 0-255
    double p, i, d;        ///< Control gains
};

_control control_az = { .input = 0, .input_prv = 0, .speed=0, .setpoint = 0,
                        .setpoint_speed = 0, .load = 0, .u = 0, .p = 8.0,
                        .i = 0.0, .d = 0.5 };
_control control_el = { .input = 0, .input_prv = 0, .speed=0, .setpoint = 0,
                        .setpoint_speed = 0, .load = 0, .u = 0, .p = 10.0,
                        .i = 0.0, .d = 0.3 };

/// Telemetry Structure for feedback
struct _telemetry{                  
  uint8_t state;
  bool az_cal;
  bool el_cal;
  uint32_t az_count;
  uint32_t el_count;
  double cur_az;
  double cur_el;
  double tar_az;
  double tar_el;
  uint8_t az_fault;
  uint8_t el_fault;
}; 

_telemetry tlm = {
  .state = 0, 
  .az_cal = false,
  .el_cal = false,
  .az_count = 0,
  .el_count = 0,
  .cur_az = 0.0,
  .cur_el = 0.0,
  .tar_az = 0.0,
  .tar_el = 0.0,
  .az_fault = 0,
  .el_fault = 0
};

enum _cmd_type {
    NONE      = 0, //special type of NONE for init and reset purposes
    STATE     = 1, // control state of rotator
    GOTOCOUNT = 2, // goto position based on motor counts
    GOTOPOS   = 4, // goto position based on angles, read tar_az and tar_el

};
//Telecomand Struct, used to pass commands from Comm.h to Main Loop
struct _telecommand{
  bool received;
  enum _cmd_type type;
  enum _rotator_state state;
  enum _control_mode control_mode; 
  int32_t tar_az_cnt;
  int32_t tar_el_cnt;
  double tar_az;
  double tar_el;
};

_telecommand cmd = {
  .received = false,
  .type = NONE,
  .state = boot,
  .control_mode = position,
  .tar_az_cnt = 0,
  .tar_el_cnt = 0,
  .tar_az = 0.0,
  .tar_el = 0.0,
};

//----GLOBAL VARIABLES------
volatile uint8_t el_fault = 0; //fault state for el
volatile uint8_t az_fault = 0; //fault state for az
uint8_t toggle_green = 0; //state for green LED
uint8_t toggle_red = 0; //state for red LED
String serIn = "";
bool disableFlag = false; //comm-> main sets state to disable


#endif /* LIBRARIES_GLOBALS_H_ */