
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <AccelStepper.h>
#include "globals.h"
#include "Comm.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x60); 

Adafruit_StepperMotor *az_motor = AFMS.getStepper(200, 1); //Stepper 1 is Azimuth
Adafruit_StepperMotor *el_motor = AFMS.getStepper(200, 2); //Stepper 2 is Elevation

void az_step_fwd(){
  az_motor->onestep(BACKWARD, MICROSTEP);
}
void az_step_rev(){
  az_motor->onestep(FORWARD, MICROSTEP);
}
void el_step_fwd(){
  el_motor->onestep(FORWARD, MICROSTEP);
}
void el_step_rev(){
  el_motor->onestep(BACKWARD, MICROSTEP);
}

AccelStepper stepper_az(az_step_fwd, az_step_rev);
AccelStepper stepper_el(el_step_fwd, el_step_rev);

Comm comm;

#define LEDGreen 8 //Green, Driver Enabled, Motion possible
#define LEDRed 13  //Red, Driver Enabled, Motion Happening

uint32_t home_time = 0;
uint32_t az_toggle_time = 0; 
uint32_t el_toggle_time = 0;

void setup() {
  // Configure LEDs
  pinMode(LEDGreen, OUTPUT);
  digitalWrite(LEDGreen, LOW);
  pinMode(LEDRed, OUTPUT);
  digitalWrite(LEDRed, LOW);

  setStateBoot();
  comm.init(); //initialize Serial Communications
  AFMS.begin(); //Start Motor Shield

  initialize_motors();

  setStateDisabled();
}

void loop() {
  // put your main code here, to run repeatedly:
  comm.proc();
  // Get position of both axis
  control_az.input = step2degAz(stepper_az.currentPosition());
  control_el.input = step2degEl(stepper_el.currentPosition());

  updateTelemetry();
  if (cmd.received == true){
    processCommand();
  }
  if (rotator.rotator_state == homing){
    doHoming();
    if (rotator.az_calibrated == true && rotator.el_calibrated ==true){
      setStateIdle();
    }
  }
  else if (rotator.rotator_state == active){
    doActive();
  }
}

void doActive(){
  if (rotator.az_calibrated == true && rotator.el_calibrated == true){
    stepper_az.moveTo(deg2stepAz(control_az.setpoint));
    stepper_el.moveTo(deg2stepEl(control_el.setpoint));
    rotator.motion_flag = true;
    stepper_az.run();
    stepper_el.run();
    if (stepper_az.distanceToGo() == 0 && stepper_el.distanceToGo() == 0) {
      setStateIdle();
    }
  }
}

void initialize_motors(){
  stepper_az.setMaxSpeed(az_cfg.maxSpeed);
  stepper_az.setAcceleration(az_cfg.maxAccel);
  stepper_az.setMinPulseWidth(az_cfg.minPulseWidth);
  // stepper1.moveTo(24);
  stepper_el.setMaxSpeed(el_cfg.maxSpeed);
  stepper_el.setAcceleration(el_cfg.maxAccel);
  stepper_el.setMinPulseWidth(el_cfg.minPulseWidth);
}

void processCommand(){
  if (cmd.type == STATE){
    if (cmd.state == disabled){
      setStateDisabled();
    }
    else if (cmd.state == idle){
      setStateIdle();
    }
    else if (cmd.state == homing){
      setStateHoming();
    }
  }
  else if (cmd.type == GOTOPOS){
    Serial.println("ping");
    control_az.setpoint = cmd.tar_az;
    control_el.setpoint = cmd.tar_el;
    // Serial.print("DEBUG GOTO: ");
    // Serial.print(control_az.setpoint); Serial.print(",");
    // Serial.print(deg2stepAz(control_az.setpoint)); Serial.print(",");
    // Serial.print(control_el.setpoint); Serial.print(",");
    // Serial.println(deg2stepAz(control_el.setpoint));
    setStateActive();
  }
  resetCommandStruct();
}

void resetCommandStruct(){
  cmd.received = false;
}

void updateTelemetry(){
  tlm.state = rotator.rotator_state;
  tlm.motion = rotator.motion_flag;
  tlm.az_cal = rotator.az_calibrated;
  tlm.el_cal = rotator.az_calibrated;
  tlm.az_count = stepper_az.currentPosition();
  tlm.el_count = stepper_el.currentPosition();
  tlm.az_togo = stepper_az.distanceToGo();
  tlm.el_togo = stepper_el.distanceToGo();
  tlm.cur_az = step2degAz(tlm.az_count);
  tlm.cur_el = step2degEl(tlm.el_count);
  tlm.tar_az = control_az.input;
  tlm.tar_el = control_el.input;  
}


//-------STATE MACHINE FUNCTONS--------------------
//STATE: 0=BOOT, 1=DISABLED, 2=IDLE, 3=EL_CAL, 4=AZ_CAL, 5=ACTIVE, 6=FAULT
/*
boot     - initial bootup of microcontroller
disabled - motor driver disabled, timers disabled, calibration lost
idle     - motor driver enabled, timers disabled.
homing   - calibration of az/el position at boot
active   - motors moving to target
error    - motor in error state....do something to revcover
*/
void setStateBoot(){
  rotator.rotator_state = boot;
  digitalWrite(LEDGreen, HIGH);
  digitalWrite(LEDRed, LOW);
}

void setStateDisabled(){
  rotator.rotator_state = disabled;
  az_motor->release();
  rotator.az_calibrated = false;
  el_motor->release();
  rotator.el_calibrated = false;
  rotator.motion_flag = false;
  digitalWrite(LEDGreen, LOW);
  digitalWrite(LEDRed, LOW);
  // cmd.state = boot; //reset cmd boot is special case
}

void setStateIdle(){
  //Motor Driver Enabled
  //Calibration completed
  digitalWrite(LEDGreen, HIGH);
  digitalWrite(LEDRed, LOW);
  rotator.rotator_state = idle;
  rotator.motion_flag = false;
  // cmd.setState = boot; //reset cmd boot is special case
}

void setStateHoming(){
  rotator.rotator_state = homing;
  stepper_az.setMaxSpeed(az_cfg.homeSpeed);
  stepper_el.setMaxSpeed(el_cfg.homeSpeed);
  home_time = millis();
  az_toggle_time = millis();
  el_toggle_time = millis();
  doHoming();
  rotator.motion_flag = true;
}

void doHoming(){
  // Do Azimuth Homing
  if (millis() - home_time < az_cfg.homeDelay) {
    if (millis() - az_toggle_time > 500){
      toggle_led_r();
      az_toggle_time = millis();
      Serial.print("AZ,"); 
      Serial.print(stepper_az.currentPosition()); Serial.print(",");
      Serial.print(az_cfg.homeDelay-(millis() - home_time)); Serial.print(",");
      Serial.println(stepper_az.distanceToGo());
    }
    // Move motors to "seek" position
    stepper_az.moveTo(stepper_az.currentPosition()-az_cfg.homeStep);
    stepper_az.run();
  }
  else{ //past home time
    stepper_az.moveTo(stepper_az.currentPosition());
    stepper_az.setCurrentPosition(deg2stepAz(az_cfg.minPos));
    control_az.setpoint = az_cfg.minPos;
    stepper_az.setMaxSpeed(az_cfg.maxSpeed);
    rotator.az_calibrated = true;
  }
  // Do Elevation Homing
  if (millis() - home_time < el_cfg.homeDelay) {
    if (millis() - el_toggle_time > 500){
      toggle_led_g();
      el_toggle_time = millis();
      Serial.print("EL,"); 
      Serial.print(stepper_el.currentPosition()); Serial.print(",");
      Serial.print(el_cfg.homeDelay-(millis() - home_time)); Serial.print(",");
      Serial.println(stepper_el.distanceToGo());
    }
    // Move motors to "seek" position
    stepper_el.moveTo(stepper_el.currentPosition()-el_cfg.homeStep);
    stepper_el.run();
  }
  else{ //past home time
    stepper_el.moveTo(stepper_el.currentPosition());
    stepper_el.setCurrentPosition(deg2stepEl(el_cfg.minPos));
    control_el.setpoint = el_cfg.minPos;
    stepper_el.setMaxSpeed(el_cfg.maxSpeed);
    rotator.el_calibrated = true;
  }
}

void setStateActive(){
  stepper_az.setMaxSpeed(az_cfg.maxSpeed);
  stepper_az.setAcceleration(az_cfg.maxAccel);
  stepper_el.setMaxSpeed(el_cfg.maxSpeed);
  stepper_el.setAcceleration(el_cfg.maxAccel);
  digitalWrite(LEDGreen, HIGH);
  digitalWrite(LEDRed, HIGH);
  rotator.rotator_state = active;
}

/**************************************************************************/
/*!
    @brief    Convert degrees to steps according to step/revolution, rotator
              gear box ratio and microstep
    @param    deg
              Degrees in float format
    @return   Steps for stepper motor driver, int32_t
*/
/**************************************************************************/
int32_t deg2stepAz(float deg) {
    return (az_cfg.gRatio * az_cfg.SPR * deg / 360);
}

/**************************************************************************/
/*!
    @brief    Convert degrees to steps according to step/revolution, rotator
              gear box ratio and microstep
    @param    deg
              Degrees in float format
    @return   Steps for stepper motor driver, int32_t
*/
/**************************************************************************/
int32_t deg2stepEl(float deg) {
    return (el_cfg.gRatio * el_cfg.SPR * deg / 360);
}

/**************************************************************************/
/*!
    @brief    Convert steps to degrees according to step/revolution, rotator
              gear box ratio and microstep
    @param    step
              Steps in int32_t format
    @return   Degrees in float format
*/
/**************************************************************************/
float step2degAz(int32_t step) {
    return (360.00 * step / (az_cfg.SPR * az_cfg.gRatio));
}
/**************************************************************************/
/*!
    @brief    Convert steps to degrees according to step/revolution, rotator
              gear box ratio and microstep
    @param    step
              Steps in int32_t format
    @return   Degrees in float format
*/
/**************************************************************************/
float step2degEl(int32_t step) {
    return (360.00 * step / (el_cfg.SPR * el_cfg.gRatio));
}

void toggle_led_r(){
  toggle_red = ~toggle_red;
  digitalWrite(LEDRed, toggle_red);
  //digitalWrite(GREEN_LED, led_state);
}

void toggle_led_g(){
  toggle_green = ~toggle_green;
  digitalWrite(LEDGreen, toggle_green);
  //digitalWrite(GREEN_LED, led_state);
}