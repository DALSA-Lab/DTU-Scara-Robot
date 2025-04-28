#include <UstepperS32.h>

#include <Wire.h>
#include "joint.h"

#define J3
#include "configuration.h"


UstepperS32 stepper;
static uint8_t state = 0x00;
static uint8_t driveCurrent, holdCurrent;
static uint8_t isHomed = 0;
static uint8_t isStallguardEnabled = 0;

uint8_t reg = 0;
uint8_t rx_buf[MAX_BUFFER] = { 0 };
uint8_t tx_buf[MAX_BUFFER + RFLAGS_SIZE] = { 0 };
bool tx_data_ready = 0;
bool rx_data_ready = 0;

size_t tx_length = 0;
size_t rx_length = 0;


/**
 * Handles commands, is called from the main loop since it contains blocking function calls which can not be called from the I2C ISR.
 * @param reg command code
 */
void stepper_receive_handler(uint8_t reg);

/**
 * Handles read request, is called from the I2C ISR since reads from the stepper are non-blocking. Also Handling reads and the subsequent wire.write(), did not work from the main loop.
 * @param reg command code
 */
void stepper_request_handler(uint8_t reg);

void receiveEvent(int n) {
  Serial.println("receive");

  reg = Wire.read();
  rx_data_ready = 1;
  Serial.println(reg);
  int i = 0;
  while (Wire.available()) {
    rx_buf[i] = Wire.read();
    // Serial.println(rx_buf[i]);
    i++;
  }
  rx_length = i;
  if (i) { DUMP_BUFFER(rx_buf, rx_length); }
}

void requestEvent() {
  Serial.println("request");
  stepper_request_handler(reg);
  tx_buf[tx_length++] = state;
  DUMP_BUFFER(tx_buf, tx_length);
  Wire.write(tx_buf, tx_length);
  // TODO: consider checking for rx_ready flag
}

void stepper_receive_handler(uint8_t reg) {
  switch (reg) {
    case SETUP:
      {
        Serial.print("Executing SETUP\n");
        memcpy(&driveCurrent, rx_buf, 1);
        memcpy(&holdCurrent, rx_buf + 1, 1);
        stepper.setup(CLOSEDLOOP, 200);
        stepper.enableClosedLoop();
        stepper.setMaxAcceleration(MAXACCEL);  //use an acceleration of 2000 fullsteps/s^2
        stepper.setMaxDeceleration(MAXACCEL);
        stepper.setMaxVelocity(MAXVEL);   //Max velocity of 800 fullsteps/s
        stepper.setControlThreshold(15);  //Adjust the control threshold - here set to 15 microsteps before making corrective action
        stepper.setCurrent(driveCurrent);
        stepper.setHoldCurrent(holdCurrent);

        isStallguardEnabled = 0;
        break;
      }

    case SETRPM:
      {
        Serial.print("Executing SETRPM\n");
        float v;
        readValue<float>(v, rx_buf, rx_length);
        stepper.setRPM(v);
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
        if (!(state & (1 << 0))) {
          stepper.moveToAngle(v);
        }

        break;
      }



      // case RUNCOTINOUS:
      //   Serial.print("Executing RUNCOTINOUS\n");
      //   break;



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

      // case SETMAXACCELERATION:
      //   Serial.print("Executing SETMAXACCELERATION\n");
      //   break;

      // case SETMAXDECELERATION:
      //   Serial.print("Executing SETMAXDECELERATION\n");
      //   break;

      // case SETMAXVELOCITY:
      //   Serial.print("Executing SETMAXVELOCITY\n");
      //   break;

    case ENABLESTALLGUARD:
      {
        Serial.print("Executing ENABLESTALLGUARD\n");
        int8_t sensitivity;
        readValue<int8_t>(sensitivity, rx_buf, rx_length);
        stepper.encoder.encoderStallDetectSensitivity = sensitivity * 1.0 / 10;
        stepper.encoder.encoderStallDetectEnable = 1;
        stepper.encoder.encoderStallDetect = 0;
        state &= ~(1 << 0);  // Clear STALL bit
        isStallguardEnabled = 1;

        break;
      }

      // case DISABLESTALLGUARD:
      //   Serial.print("Executing DISABLESTALLGUARD\n");
      //   break;

      // case CLEARSTALL:
      //   Serial.print("Executing CLEARSTALL\n");
      //   break;



    case SETBRAKEMODE:
      {
        Serial.print("Executing SETBRAKEMODE\n");
        uint8_t v;
        readValue<uint8_t>(v, rx_buf, rx_length);
        stepper.setBrakeMode(v);
        break;
      }

    case ENABLEPID:
      Serial.print("Executing ENABLEPID\n");
      break;

    case DISABLEPID:
      Serial.print("Executing DISABLEPID\n");
      break;


    case DISABLECLOSEDLOOP:
      {
        Serial.print("Executing DISABLECLOSEDLOOP\n");
        uint8_t v;
        readValue<uint8_t>(v, rx_buf, rx_length);
        stepper.disableClosedLoop();
        break;
      }
      // case SETCONTROLTHRESHOLD:
      //   Serial.print("Executing SETCONTROLTHRESHOLD\n");
      //   break;
      // case MOVETOEND:
      //   Serial.print("Executing MOVETOEND\n");
      //   break;

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
        int8_t sensitivity;
        uint8_t current;
        memcpy(&dir, rx_buf, 1);
        memcpy(&speed, rx_buf + 1, 1);
        memcpy(&sensitivity, rx_buf + 2, 1);
        memcpy(&current, rx_buf + 3, 1);

        stepper.stop();
        stepper.encoder = TLE5012B();  // Reset Enocoder to clear stall
        stepper.encoder.encoderStallDetect = 0;

        stepper.setRPM(dir ? speed : -speed);
        stepper.setCurrent(current);
        stepper.encoder.encoderStallDetectSensitivity = sensitivity * 1.0 / 10;
        stepper.encoder.encoderStallDetectEnable = 1;

        while (!stepper.encoder.encoderStallDetect) {
          delay(5);
        }
        stepper.encoder.setHome();
        stepper.stop();  // Stop motor !
        stepper.encoder.encoderStallDetectEnable = 0;

        stepper.setCurrent(driveCurrent);

        isHomed = 1;
        break;
      }

    default:
      Serial.println("Unknown command");
      break;
  }
}

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
      Serial.print("Executing GETDRIVERRPM\n");
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
        writeValue<uint8_t>(stepper.isStalled(), tx_buf, tx_length);

        tx_data_ready = 1;
        break;
      }
    case ISHOMED:
      {
        Serial.print("Executing ISHOMED\n");
        writeValue<uint8_t>(isHomed, tx_buf, tx_length);
        tx_data_ready = 1;
        break;
      }

      // case GETPIDERROR:
      //   Serial.print("Executing GETPIDERROR\n");
      //   break;

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


void setup(void) {
  // Join I2C bus as follower
  Wire.begin(ADR);

  // stepper.setup(CLOSEDLOOP, 200);  //Initialize uStepper S32 to use closed loop control with 200 steps per revolution motor - i.e. 1.8 deg stepper

  // For the closed loop position control the acceleration and velocity parameters define the response of the control:

  // stepper.setControlThreshold(15);  //Adjust the control threshold - here set to 15 microsteps before making corrective action

  // stepper.enableStallguard(10, true, 60);
  // stepper.disableStallguard();

  // stepper.encoder.encoderStallDetectSensitivity = 1;  //Encoder stalldetect sensitivity - From -10 to 1 where lower number is less sensitive and higher is more sensitive. -0.25 works for most.
  // stepper.encoder.encoderStallDetectEnable = 1;           //Enable the encoder stall detect

  Serial.begin(9600);

  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop(void) {

  state |= isStallguardEnabled ? stepper.encoder.encoderStallDetect << 0 : 0 << 0;

  if (rx_data_ready) {
    rx_data_ready = 0;
    state |= 1 << 1;  // set is busy flag
    stepper_receive_handler(reg);
    state &= ~(1 << 1);  // reset is busy flag
  }

  delay(1);
}
