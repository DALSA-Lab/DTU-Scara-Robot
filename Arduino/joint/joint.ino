/**
 * @file joint.ino
 * @author Sebastian Storz
 * @brief joint firmware
 * @version 0.1
 * @date 2025-05-27
 *
 * @copyright Copyright (c) 2025
 *
 * This file contains the joint firmware.
 *
 */

#include <UstepperS32.h>

#include <Wire.h>
#include "joint.h"

/**
 * @brief Define either joint that is to be flashed
 * 
 * Define either J1, J2, J3 or J4 and subsequently include configuration.h 
 */
#define J1
#include "configuration.h"


UstepperS32 stepper;
static uint8_t state = 0x00;
static uint8_t driveCurrent, holdCurrent;
static uint8_t isHomed = 0;
static uint8_t isStalled = 0;
static uint8_t isSetup = 0;
static uint8_t isStallguardEnabled = 0;
static int stallguardThreshold = 100;

uint8_t reg = 0;
uint8_t rx_buf[MAX_BUFFER] = { 0 };
uint8_t tx_buf[MAX_BUFFER + RFLAGS_SIZE] = { 0 };
bool tx_data_ready = 0;
bool rx_data_ready = 0;

size_t tx_length = 0;
size_t rx_length = 0;


void stepper_receive_handler(uint8_t reg);
void stepper_request_handler(uint8_t reg);

/**
 * @brief I2C receive event Handler.
 *
 * Reads the content of the received message. Saves the reg so it can be used in the main loop. If the master invokes the read() function the message contains only the register byte 
 * and no payload. If the master invokes the write() the message has a payload of appropriate size for the command.
 * For a read request the message looks like this: \n 
 * \< [REG] \n 
 * \> [TXBUFn]...[TXBUF2][TXBUF1][TXBUF0][FLAGS] \n 
 * For a command the message looks like this: \n 
 * \< [REG][RXBUFn]...[RXBUF2][RXBUF1][RXBUF0] \n 
 * \> [FLAGS] \n 
 * The payload is read into the rx_buf, rx_length is set to the payload length and the rx_data_ready flag is set.
 * @param n the number of bytes read from the controller device: MAX_BUFFER
 */
void receiveEvent(int n) {
  // Serial.println("receive");
  reg = Wire.read();

  // Serial.println(reg);
  int i = 0;
  while (Wire.available()) {
    rx_buf[i] = Wire.read();
    i++;
  }
  rx_length = i;
  rx_data_ready = 1;
  // if (i) { DUMP_BUFFER(rx_buf, rx_length); }
}

/**
 * @brief I2C request event Handler.
 *
 * Sends the response data to the master. Every transaction begins with a receive event. This function is only called when the master calls the read() function.
 * Hence this function is only invoked after the receiveEvent() handler has been called. The function calls the stepper_request_handler() which is non-blocking.
 * stepper_request_handler() populates the tx_buf, the current state flags are appended to the tx_buf and then it is send to the master.
 */
void requestEvent() {
  // Serial.println("request");
  stepper_request_handler(reg);
  tx_buf[tx_length++] = state;
  // DUMP_BUFFER(tx_buf, tx_length);
  Wire.write(tx_buf, tx_length);
}

/**
 * @brief Handles commands received via I2C.
 * @warning This is a blocking function which may take some time to execute. This function must not be called from an ISR or callback! 
 * Call from main loop instead.

 * All the registers that are inside this handler are considered command which require an action. 
 * @param reg command that should be executed.
 */
void stepper_receive_handler(uint8_t reg) {
  switch (reg) {
    case SETUP:
      {
        Serial.print("Executing SETUP\n");
        memcpy(&driveCurrent, rx_buf, 1);
        memcpy(&holdCurrent, rx_buf + 1, 1);
        if (!isSetup) {
          stepper.setup(CLOSEDLOOP, 200);
          isHomed = 0;
        }
        
        stepper.setMaxAcceleration(MAXACCEL);
        stepper.setMaxDeceleration(MAXACCEL);
        stepper.setMaxVelocity(MAXVEL);
        stepper.setControlThreshold(15);  //Adjust the control threshold - here set to 15 microsteps before making corrective action
        stepper.setCurrent(driveCurrent);
        stepper.setHoldCurrent(holdCurrent);
        stepper.moveToAngle(stepper.angleMoved());
        stepper.enableClosedLoop();
        stepper.stop();

        isStallguardEnabled = 0;
        isSetup = 1;
        isStalled = 0;
        break;
      }

    case SETRPM:
      {
        Serial.print("Executing SETRPM\n");
        float v;
        readValue<float>(v, rx_buf, rx_length);
        if (!isStalled) {
          stepper.setRPM(v);
        }
        break;
      }

    case MOVESTEPS:
      {
        Serial.print("Executing MOVESTEPS\n");
        int32_t v;
        readValue<int32_t>(v, rx_buf, rx_length);
        stepper.moveSteps(v);

        break;
      }

    case MOVETOANGLE:
      {
        Serial.print("Executing MOVETOANGLE\n");
        float v;
        readValue<float>(v, rx_buf, rx_length);
        // Serial.println(v);
        if (!isStalled) {
          stepper.moveToAngle(v);
        }

        break;
      }


    case SETCURRENT:
      {
        Serial.print("Executing SETCURRENT\n");
        uint8_t v;
        readValue<uint8_t>(v, rx_buf, rx_length);
        stepper.setCurrent(v);
        break;
      }

    case SETHOLDCURRENT:
      {
        Serial.print("Executing SETHOLDCURRENT\n");
        uint8_t v;
        readValue<uint8_t>(v, rx_buf, rx_length);
        stepper.setHoldCurrent(v);
        break;
      }

      case SETMAXACCELERATION:
      {
        Serial.print("Executing SETMAXACCELERATION\n");
        float v;
        readValue<float>(v, rx_buf, rx_length);
        stepper.setMaxAcceleration(v*200.0/360.0);
        stepper.setMaxDeceleration(v*200.0/360.0);
        break;
      }

      // case SETMAXDECELERATION:
      // Serial.print("Executing SETMAXDECELERATION\n");
      //   break;

      case SETMAXVELOCITY:
      {
        Serial.print("Executing SETMAXVELOCITY\n");
        float v;
        readValue<float>(v, rx_buf, rx_length);
        stepper.setMaxVelocity(v*200.0/360.0);
        break;
      }

    case ENABLESTALLGUARD:
      {
        Serial.print("Executing ENABLESTALLGUARD\n");

        // Very simple workaround for stall detection, since the built-in encoder stall-detection is tricky to work with in particular in combination with homeing since it can not be reset.
        uint8_t sensitivity;
        readValue<uint8_t>(sensitivity, rx_buf, rx_length);
        stallguardThreshold = sensitivity * 10;
        // // Serial.println(sensitivity*1.0/10);
        // stepper.encoder.encoderStallDetectSensitivity = sensitivity * 1.0/10 ;
        // stepper.encoder.encoderStallDetectEnable = 1;
        // stepper.encoder.encoderStallDetect = 0;
        state &= ~(1 << 0);  // Clear STALL bit
        isStallguardEnabled = 1;
        isStalled = 0;

        break;
      }

      // case DISABLESTALLGUARD:
      // Serial.print("Executing DISABLESTALLGUARD\n");
      //   break;

      // case CLEARSTALL:
      // Serial.print("Executing CLEARSTALL\n");
      //   break;



    case SETBRAKEMODE:
      {
        Serial.print("Executing SETBRAKEMODE\n");
        uint8_t v;
        readValue<uint8_t>(v, rx_buf, rx_length);
        stepper.setBrakeMode(v);
        break;
      }

      // case ENABLEPID:
      //   // Serial.print("Executing ENABLEPID\n");
      //   break;

      // case DISABLEPID:
      //   // Serial.print("Executing DISABLEPID\n");
      //   break;


    case DISABLECLOSEDLOOP:
      {
        Serial.print("Executing DISABLECLOSEDLOOP\n");
        uint8_t v;
        readValue<uint8_t>(v, rx_buf, rx_length);
        stepper.disableClosedLoop();
        break;
      }


    case STOP:
      {
        Serial.print("Executing STOP\n");
        uint8_t v;
        readValue<uint8_t>(v, rx_buf, rx_length);
        stepper.stop(v);
        break;
      }

    case CHECKORIENTATION:
      {
        Serial.print("Executing CHECKORIENTATION\n");
        float v;
        readValue<float>(v, rx_buf, rx_length);
        stepper.checkOrientation(v);
        break;
      }

    case HOME:
      {
        Serial.print("Executing HOME\n");

        uint8_t dir;
        uint8_t speed;
        uint8_t sensitivity;
        uint8_t current;
        memcpy(&dir, rx_buf, 1);
        memcpy(&speed, rx_buf + 1, 1);
        memcpy(&sensitivity, rx_buf + 2, 1);
        memcpy(&current, rx_buf + 3, 1);

        stepper.stop();
        // stepper.encoder = TLE5012B();  // Reset Enocoder to clear stall<<
        // stepper.encoder.init();
        // stepper.encoder.encoderStallDetect = 0;<<

        stepper.setRPM(dir ? speed : -speed);
        stepper.setCurrent(current);
        // stepper.encoder.encoderStallDetectSensitivity = sensitivity * 1.0 / 10;<<
        // stepper.encoder.encoderStallDetectEnable = 1;<<

        float err;
        do {
          err = stepper.getPidError();

          Serial.println(abs(err));
          delay(1);
        } while (abs(err) < sensitivity);

        // while (!stepper.encoder.encoderStallDetect) {
        //   delay(5);
        // }
        stepper.encoder.setHome();
        stepper.driver.setHome();
        stepper.stop();  // Stop motor !

        // stepper.encoder = TLE5012B();  // Reset Enocoder to clear stall
        // stepper.encoder.encoderStallDetect = 0;
        // stepper.encoder.setHome();
        // stepper.encoder.encoderStallDetectEnable = 0;<<

        stepper.setCurrent(driveCurrent);

        isHomed = 1;
        isStalled = 0;
        break;
      }

    default:
      Serial.println("Unknown command");
      break;
  }
}

/**
 * @brief Handles read request received via I2C.

 * Can be invoked from the I2C ISR since reads from the stepper are non-blocking. 
 * Also Handling reads and the subsequent wire.write(), did not work from the main loop.

 * All registers inside this function are regarded as read only.
 * @param reg register to read.
 */

void stepper_request_handler(uint8_t reg) {
  switch (reg) {
    case PING:
      {
        Serial.print("Executing PING\n");
        writeValue<char>(ACK, tx_buf, tx_length);
        tx_data_ready = true;
        break;
      }

    case GETDRIVERRPM:
      // Serial.print("Executing GETDRIVERRPM\n");
      break;



    case ANGLEMOVED:
      {
        Serial.print("Executing ANGLEMOVED\n");
        writeValue<float>(stepper.angleMoved(), tx_buf, tx_length);
        tx_data_ready = 1;
        break;
      }

    case ISSTALLED:
      {
        Serial.print("Executing ISSTALLED\n");
        writeValue<uint8_t>(state & 0x01, tx_buf, tx_length);

        tx_data_ready = 1;
        break;
      }
    case ISHOMED:
      {
        Serial.print("Executing ISHOMED\n");
        writeValue<uint8_t>(isHomed ? 1 : 0, tx_buf, tx_length);
        tx_data_ready = 1;
        break;
      }

    case ISSETUP:
      {
        Serial.print("Executing ISSETUP\n");
        writeValue<uint8_t>(isSetup ? 1 : 0, tx_buf, tx_length);
        tx_data_ready = 1;
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
      Serial.println("Unknown function");
      // Instead of sending a zero buffer, set the tx_length to 0 to only send return flags
      tx_length = 0;
      break;
  }
}

/**
 * @brief Setup Peripherals

 * Setup I2C with the address ADR, and begin Serial for debugging with baudrate 9600.
 */
void setup(void) {
  // Join I2C bus as follower
  Wire.begin(ADR);
  Serial.begin(9600);

  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

/**
 * @brief Main loop

 * Executes the following: \n 
 * 1) if isStallguardEnabled: compares stepper.getPidError() with stallguardThreshold and sets BIT0 of the state byte. \n 
 * 2) sets/clears BIT2 of the state byte if the joint is homed or not. \n 
 * 3) sets/clears BIT3 of the state byte if the joint is setup or not. \n 
 * 4) if rx_data_ready: set BIT1 of the state byte to indicate device is busy. Invoke stepper_receive_handler. 
 * Clear BIT1 of the state byte to indicate device is no longer busy \n 
 * @todo
 - why are BIT2 and BIT3 constantly checked and set? Would it be sufficient to do this only invoking the actual functions?
 */
void loop(void) {

  if (isStallguardEnabled) {
    float err = stepper.getPidError();
    if (abs(err) > stallguardThreshold) {
      isStalled = 1;
      state |= (1 << 0);
      stepper.stop(SOFT);  // UNTESTED
    } else if (!isStalled) {
      state &= ~(1 << 0);
    }
    // state |= (abs(err) > stallguardThreshold) ? 0x01 : 0x00;
    Serial.print(abs(err));
    Serial.print("\t");
    Serial.println(state);
  }

  isHomed ? state |= (1 << 2) : state &= ~(1 << 2);
  isSetup ? state |= (1 << 3) : state &= ~(1 << 3);

  if (rx_data_ready) {
    rx_data_ready = 0;
    state |= 1 << 1;  // set is busy flag
    stepper_receive_handler(reg);
    state &= ~(1 << 1);  // reset is busy flag
  }

  delay(10);
}
