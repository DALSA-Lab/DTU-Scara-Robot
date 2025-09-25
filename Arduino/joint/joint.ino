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
#define J4
#include "configuration.h"


UstepperS32 stepper;
static uint8_t driveCurrent, holdCurrent;
static uint8_t notHomed = 1;
static uint8_t isStalled = 0;
static uint8_t isBusy = 0;
static uint8_t notSetup= 1;
static uint8_t isStallguardEnabled = 0;
static int stallguardThreshold = 100;

uint8_t reg = 0;
uint8_t rx_buf[MAX_BUFFER] = { 0 };
uint8_t tx_buf[MAX_BUFFER + RFLAGS_SIZE] = { 0 };
bool tx_data_ready = 0;
bool rx_data_ready = 0;

size_t tx_length = 0;
size_t rx_length = 0;


void blocking_handler(uint8_t reg);
void non_blocking_handler(uint8_t reg);

/**
 * @brief I2C receive event Handler.
 *
 * Reads the content of the received message. Saves the register so it can be used in the main loop. 
 * If the master invokes the read() function the message contains only the register byte and no payload.
 * If the master invokes the write() the message has a payload of appropriate size for the command.
 * Every I2C transaction starts with a receive event when the command is sent and is immediatly followed by a request 
 * since at minimum the flags need to be transmitted back. This means that the receive handler and request handler are always 
 * executed sequentially. The main loop is not executed since both handlers are ISRs.
 * For a read request the message looks like this: \n 
 * \< [REG] \n 
 * \> [TXBUFn]...[TXBUF2][TXBUF1][TXBUF0][FLAGS] \n 
 * For a command the message looks like this: \n 
 * \< [REG][RXBUFn]...[RXBUF2][RXBUF1][RXBUF0] \n 
 * \> [FLAGS] \n 
 * The payload is read into the rx_buf, rx_length is set to the payload length.
 * @param n the number of bytes read from the controller device: MAX_BUFFER
 */
void receiveEvent(int n) {
  Serial.print("receive: \t");
  reg = Wire.read();

  Serial.print("Register: ");
  Serial.println(reg);
  int i = 0;
  while (Wire.available()) {
    rx_buf[i] = Wire.read();
    i++;
  }
  rx_length = i;
  if (i) { DUMP_BUFFER(rx_buf, rx_length); }
}

/**
 * @brief I2C request event Handler.
 *
 * Sends the response data to the master. Every transaction begins with a receive event. The request event is always triggered since at a minimum the status flags are returned
 * to the master.
 * Hence this function is only invoked after the receiveEvent() handler has been called. The function calls the non_blocking_handler() which is non-blocking.
 * Since most Ustepper functions are non-blocking as they just read/write registers to the stepper driver/encoder they can be handled directly in the ISR.
 * The non_blocking_handler() populates the tx_buf with relevant data, the current state flags are appended to the tx_buf and then it is send to the master.
 */
void requestEvent() {
  Serial.print("request: \t");
  Serial.print("Register: ");
  Serial.println(reg);

  non_blocking_handler(reg);
  uint8_t state = 0x00;
  state |= (isStalled << 0);
  state |= (isBusy << 1);
  state |= (notHomed << 2);
  state |= (notSetup << 3);
  tx_buf[tx_length++] = state;
  // DUMP_BUFFER(tx_buf, tx_length);
  Wire.write(tx_buf, tx_length);
}

/**
 * @brief Handles commands received via I2C.
 * @warning This is a blocking function which may take some time to execute. This function must not be called from an ISR or callback! 
 * Call from main loop instead.

 * The registers handled in this handler are those whose implementation can take time and can thereby not be called directly from the request handler.
 * @param reg command that should be executed.
 */
void blocking_handler(uint8_t reg) {
  Serial.print("Receive Handler: \t Register: ");
  Serial.println(reg);
  switch (reg) {
    case SETUP:
      {
        Serial.print("Executing SETUP\n");
        memcpy(&driveCurrent, rx_buf, 1);
        memcpy(&holdCurrent, rx_buf + 1, 1);
        if (!isSetup) {
          stepper.setup(CLOSEDLOOP, 200);
          notHomed = 1;
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
        notSetup = 0;
        isStalled = 0;
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

        notHomed = 0;
        isStalled = 0;
        break;
      }

    default:
      Serial.println("UNKOWN REGISTER");
      break;
  }
}

/**
 * @brief Handles read request received via I2C.

 * Can be invoked from the I2C ISR since reads from the stepper are non-blocking. 
 * Also Handling reads and the subsequent wire.write(), did not work from the main loop.
 * @param reg command to execute/register to read.
 */

void non_blocking_handler(uint8_t reg) {
  rx_data_ready = 0;
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
        writeValue<uint8_t>(isStalled, tx_buf, tx_length);

        tx_data_ready = 1;
        break;
      }
    case ISHOMED:
      {
        Serial.print("Executing ISHOMED\n");
        writeValue<uint8_t>(notHomed, tx_buf, tx_length);
        tx_data_ready = 1;
        break;
      }

    case ISSETUP:
      {
        Serial.print("Executing ISSETUP\n");
        writeValue<uint8_t>(notSetup, tx_buf, tx_length);
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

      /* Below are write commands that are non-blocking and can be executed from the request ISR */

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
        stepper.setMaxAcceleration(v * 200.0 / 360.0);
        stepper.setMaxDeceleration(v * 200.0 / 360.0);
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
        stepper.setMaxVelocity(v * 200.0 / 360.0);
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

    default:
      Serial.println("No data to write, sending flags");
      // If the received register is not a Write register (handled in this handler), it must be a Read register, hence set
      // rx_data_ready flag to signal mainloop to handle it.
      rx_data_ready = 1;
      // Set the tx_length to 0 to only send return flags
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
 * 1) if isStallguardEnabled: compares stepper.getPidError() with stallguardThreshold and sets isStalled flag. \n 
 * 2) sets/clears notHomed flag if the joint is homed or not. \n 
 * 3) sets/clears notSetup if the joint is setup or not. \n 
 * 4) if rx_data_ready: set isBusy flag to indicate device is busy. Invoke blocking_handler. 
 * Clear isBusy flag to indicate device is no longer busy \n 
 */
void loop(void) {

  if (isStallguardEnabled && !isStalled) {
    float err = stepper.getPidError();
    if (abs(err) > stallguardThreshold) {
      isStalled = 1;
      stepper.stop(SOFT);  // UNTESTED
    }
    // state |= (abs(err) > stallguardThreshold) ? 0x01 : 0x00;
    // Serial.print(abs(err));
    // Serial.print("\t");
    // Serial.println(state);
  }


  if (rx_data_ready) {
    rx_data_ready = 0;
    isBusy = 1;  // set is busy flag
    blocking_handler(reg);
    isBusy = 0;  // reset is busy flag
  }

  delay(10);
}
