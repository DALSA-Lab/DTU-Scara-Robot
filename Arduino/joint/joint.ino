/********************************************************************************************
* 	    File:  closedLoop.ino                                              		            *
*		Version:    2.3.0                                          						    *
*      	Date: 		October 7th, 2023  	                                    				*
*       Author:  Thomas Hørring Olsen                                                       *
*  Description:  Example sketch for closed loop position control!                           *                           *
*                This example demonstrates how easy closed loop position control can be     *
*                achieved using the uStepper S32 !                                          *
*                The only thing needed to activate closed loop control, is in the           *
*                stepper.setup() function, where the object is initiated with the keyword   *
*                "CLOSEDLOOP", followed by the number of steps per revolution setting.      *
*                                                                                           *
*                                                                                           *
* For more information, check out the documentation:                                        *
*    http://ustepper.com/docs/usteppers/html/index.html                                     *
*                                                                                           *
*                                                                                           *
*********************************************************************************************
*	(C) 2023                                                                                *
*                                                                                           *
*	uStepper ApS                                                                            *
*	www.ustepper.com                                                                        *
*	administration@ustepper.com                                                             *
*                                                                                           *
*	The code contained in this file is released under the following open source license:    *
*                                                                                           *
*			Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International         *
*                                                                                           *
* 	The code in this file is provided without warranty of any kind - use at own risk!       *
* 	neither uStepper ApS nor the author, can be held responsible for any damage             *
* 	caused by the use of the code contained in this file !                                  *
*                                                                                           *
*                                                                                           *
********************************************************************************************/

#include <UstepperS32.h>
#include <Wire.h>
#include "uSerial.h"

#define ADR 0x11
#define MAX_BUFFER 4  // Bytes


UstepperS32 stepper;

uint8_t rx_buf[MAX_BUFFER] = { 0 };
uint8_t tx_buf[MAX_BUFFER] = { 0 };
bool tx_data_ready = 0;
size_t tx_length = 0;
size_t rx_length = 0;

void stepper_reg_handler(uint8_t reg);

void receiveEvent(int n) {
  Serial.println("receive");

  uint8_t reg = Wire.read();
  Serial.println(reg);
  int i = 0;
  while (Wire.available()) {
    rx_buf[i] = Wire.read();
    Serial.println(rx_buf[i]);
    i++;
  }
  rx_length = i;
  if (i) { DUMP_BUFFER(rx_buf, rx_length); }
  stepper_reg_handler(reg);
}

void requestEvent() {
  Serial.println("request");
  if (tx_data_ready) {
    Serial.println("DEBUG: TX Data Ready, Sending Data:");
    DUMP_BUFFER(tx_buf, tx_length);
    tx_data_ready = 0;
  } else {
    Serial.println("DEBUG: TX Data not ready yet");
    memset(tx_buf, 0, MAX_BUFFER);
  }
  Wire.write(tx_buf, tx_length);
}

void stepper_reg_handler(uint8_t reg) {
  switch (reg) {
    case PING:
      {
        Serial.print("Executing PING\n");
        writeValue<char>(ACK, tx_buf, tx_length);
        tx_data_ready = true;
        break;
      }

    case SETUP:
      Serial.print("Executing SETUP\n");
      break;

    case SETRPM:
      {
        Serial.print("Executing SETRPM\n");
        float v;
        readValue<float>(v, rx_buf, rx_length);
        stepper.setRPM(v);
        break;
      }

    case GETDRIVERRPM:
      Serial.print("Executing GETDRIVERRPM\n");
      break;

      // case MOVESTEPS:
      //   {
      //     Serial.print("Executing MOVESTEPS\n");
      //     Serial2.write(ACK);  // send ACK to show that command was understood, then execute, then send
      //     int32_t v;
      //     if (readValue<int32_t>(v, rx_buf, rx_length) == 0) {
      //       stepper.moveSteps(v);
      //       Serial.println(v);
      //       Serial2.write(ACK);
      //     } else {
      //       // Send NACK
      //       Serial2.write(NACK);
      //     }

      //     break;
      //   }

      // case MOVEANGLE:
      //   {
      //     Serial.print("Executing MOVEANGLE\n");
      //     Serial2.write(ACK);  // send ACK to show that command was understood, then execute, then send
      //     break;
      //   }

    case MOVETOANGLE:
      {
        Serial.print("Executing MOVETOANGLE\n");
        float v;
        readValue<float>(v, rx_buf, rx_length);
        Serial.println(v);
        stepper.moveToAngle(v);
        break;
      }

    // case GETMOTORSTATE:
    //   Serial.print("Executing GETMOTORSTATE\n");
    //   break;

    // case RUNCOTINOUS:
    //   Serial.print("Executing RUNCOTINOUS\n");
    //   break;

    case ANGLEMOVED:
      {
        Serial.print("Executing ANGLEMOVED\n");
        writeValue<float>(stepper.angleMoved(), tx_buf, tx_length);
        tx_data_ready = 1;
        break;
      }

    // case SETCURRENT:
    //   {
    //     Serial.print("Executing SETCURRENT\n");
    //     uint8_t v;
    //     readValue<uint8_t>(v, rx_buf, rx_length);
    //     stepper.setCurrent(v);
    //     break;
    //   }

    // case SETHOLDCURRENT:
    //   {
    //     Serial.print("Executing SETHOLDCURRENT\n");
    //     uint8_t v;
    //     readValue<uint8_t>(v, rx_buf, rx_length);
    //     stepper.setHoldCurrent(v);
    //     break;
    //   }

    // case SETMAXACCELERATION:
    //   Serial.print("Executing SETMAXACCELERATION\n");
    //   break;

    // case SETMAXDECELERATION:
    //   Serial.print("Executing SETMAXDECELERATION\n");
    //   break;

    // case SETMAXVELOCITY:
    //   Serial.print("Executing SETMAXVELOCITY\n");
    //   break;

    // case ENABLESTALLGUARD:
    //   Serial.print("Executing ENABLESTALLGUARD\n");
    //   break;

    // case DISABLESTALLGUARD:
    //   Serial.print("Executing DISABLESTALLGUARD\n");
    //   break;

    // case CLEARSTALL:
    //   Serial.print("Executing CLEARSTALL\n");
    //   break;

    // case ISSTALLED:
    //   Serial.print("Executing ISSTALLED\n");
    //   break;

    // case SETBRAKEMODE:
    //   {
    //     Serial.print("Executing SETBRAKEMODE\n");
    //     uint8_t v;
    //     readValue<uint8_t>(v, rx_buf, rx_length);
    //     stepper.setBrakeMode(v);
    //     break;
    //   }

    case ENABLEPID:
      Serial.print("Executing ENABLEPID\n");
      break;

    case DISABLEPID:
      Serial.print("Executing DISABLEPID\n");
      break;

    case ENABLECLOSEDLOOP:
      Serial.print("Executing ENABLECLOSEDLOOP\n");
      break;

    // case DISABLECLOSEDLOOP:
    //   {
    //     Serial.print("Executing DISABLECLOSEDLOOP\n");
    //     Serial2.write(ACK);  // send ACK to show that command was understood, then execute, then send
    //     int8_t v;
    //     if (readValue<int8_t>(v, rx_buf, rx_length) == 0) {
    //       stepper.disableClosedLoop();
    //       Serial2.write(ACK);
    //     } else {
    //       // Send NACK
    //       Serial2.write(NACK);
    //     }
    //     break;
    //   }

    // case SETCONTROLTHRESHOLD:
    //   Serial.print("Executing SETCONTROLTHRESHOLD\n");
    //   break;
    // case MOVETOEND:
    //   Serial.print("Executing MOVETOEND\n");
    //   break;

    // case STOP:
    //   {
    //     Serial.print("Executing STOP\n");
    //     Serial2.write(ACK);  // send ACK to show that command was understood, then execute, then send
    //     int8_t v;
    //     if (readValue<int8_t>(v, rx_buf, rx_length) == 0) {
    //       stepper.stop(v);
    //       Serial2.write(ACK);
    //     } else {
    //       // Send NACK
    //       Serial2.write(NACK);
    //     }
    //     break;
    //   }
    // case GETPIDERROR:
    //   Serial.print("Executing GETPIDERROR\n");
    //   break;

    case CHECKORIENTATION:
      {
        Serial.print("Executing CHECKORIENTATION\n");
        float v;
        readValue<float>(v, rx_buf, rx_length);
        stepper.checkOrientation(v);
        break;
      }
    case GETENCODERRPM:
      {
        Serial.print("Executing GETENCODERRPM\n");
        writeValue<float>(stepper.encoder.getRPM(), tx_buf, tx_length);
        tx_data_ready = 1;
        break;
      }

    default:
      Serial.print("Unknown function\n");
      Serial2.write(NACK);
      break;
  }
}

void setup(void) {

  // Join I2C bus as follower
  Wire.begin(ADR);



  stepper.setup(CLOSEDLOOP, 200);  //Initialize uStepper S32 to use closed loop control with 200 steps per revolution motor - i.e. 1.8 deg stepper
  // stepper.checkOrientation(30.0);  //Check orientation of motor connector with +/- 30 microsteps movement

  // For the closed loop position control the acceleration and velocity parameters define the response of the control:
  stepper.setMaxAcceleration(10000);  //use an acceleration of 2000 fullsteps/s^2
  stepper.setMaxDeceleration(10000);
  stepper.setMaxVelocity(2000);     //Max velocity of 800 fullsteps/s
  stepper.setControlThreshold(15);  //Adjust the control threshold - here set to 15 microsteps before making corrective action
  stepper.setCurrent(10);
  stepper.setHoldCurrent(10);
  // stepper.enableStallguard(10, true, 2);
  stepper.disableStallguard();
  Serial.begin(9600);

  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  
  // stepper.checkOrientation(30.0);
}

void loop(void) {
  if (stepper.isStalled(10)) {
    Serial.println("STALLED");
  }
}
