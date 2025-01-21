/*!
* @file Comm.h
*
* Driver for Communications protocol as referred, in Hamlib.
*
* Licensed under the MIT
*
*/
#ifndef LIBRARIES_COMM_H_
#define LIBRARIES_COMM_H_

#include <Arduino.h>
#include "globals.h"
#define BUFFER_SIZE   256   ///< Set the size of serial buffer
#define BAUDRATE      38400 ///< Set the Baudrate of protocol

/**************************************************************************/
/*!
    @brief    Class that functions for communications implementation
*/
/**************************************************************************/

class Comm {
public:

  /**************************************************************************/
  /*!
      @brief    Initialize the USB Serial Communications
  */
  /**************************************************************************/
  void init() {
	  Serial.begin(BAUDRATE);
    // while (!Serial) {
    //   ;  // wait for serial port to connect. Needed for native USB port only
    // }
    // Serial.println("Start...");
  }

  /**************************************************************************/
  /*!
      @brief    Get the commands and send responses to client
  */
  /**************************************************************************/
  void proc() {
    int serLen = Serial.available();
     if (serLen > 0){
      for (int i = 0; i < serLen; i++){
        char c = Serial.read();          // gets one byte from the network buffer
        serIn += c;
      } 
      serIn.trim(); 
      serIn.toUpperCase();
      if (serIn.length() > 0){
        cmd.received = true;
        processSerial(serIn);
      }
    }
    // Serial.println(serIn);
    serIn = "";
  }

  /**************************************************************************/
  /*!
      @brief    If there is input on the serial port process it
  */
  /**************************************************************************/
  void processSerial(String in){
    if (in.charAt(0) == 'Q'){//query
      printTlmString(in.charAt(0));
      cmd.received = false;
    }
    else if (in[0] == 'D'){//Disable
      cmd.type = STATE;
      cmd.state = disabled;
      Serial.print("$"), Serial.print(in.charAt(0)); 
      Serial.print(","); Serial.println(millis());
    }
    else if (in[0] == 'I'){//Idle
      cmd.type = STATE;
      cmd.state = idle;
      Serial.print("$"), Serial.print(in.charAt(0));
      Serial.print(","); Serial.println(millis());
    }
    else if (in[0] == 'H'){//Disable
      cmd.type = STATE;
      cmd.state = homing;
      Serial.print("$"), Serial.print(in.charAt(0));
      Serial.print(","); Serial.println(millis());
    }
    else if (in[0] == 'P'){//PARK
      cmd.type = GOTOPOS;
      Serial.print("$"), Serial.print(in.charAt(0));
      Serial.print(","); Serial.print(cmd.type); 
      Serial.print(","); Serial.println(millis());
      cmd.tar_az = rotator.park_az;
      cmd.tar_el = rotator.park_el;
    }
    else if (in[0] == 'A' && in[1] == 'Z'){//GOTO AZ EL
      cmd.type = GOTOPOS;
      // control_az.setpoint = 0;
    }
    else if (in[0] == 'V'){
      //velocity command
    }
    else {
      Serial.print("$"),
      Serial.print(in),
      Serial.println("?");
    }
  }

  void printTlmString(char in){
    Serial.print("$"), Serial.print(in); Serial.print(",");
    Serial.print(millis()); Serial.print(",");
    Serial.print(tlm.state); Serial.print(",");
    Serial.print(tlm.motion); Serial.print(",");
    Serial.print(tlm.az_cal); Serial.print(",");
    Serial.print(tlm.el_cal); Serial.print(",");
    Serial.print(tlm.az_count); Serial.print(",");
    Serial.print(tlm.el_count); Serial.print(",");
    Serial.print(tlm.cur_az); Serial.print(",");
    Serial.print(tlm.cur_el); Serial.print(",");
    Serial.print(tlm.tar_az); Serial.print(",");
    Serial.print(tlm.tar_el); Serial.print(",");
    Serial.print(tlm.az_fault); Serial.print(",");
    Serial.print(tlm.el_fault);
    Serial.println();
  };
};

#endif /* LIBRARIES_COMM_H_ */