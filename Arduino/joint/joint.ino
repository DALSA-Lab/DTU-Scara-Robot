#include <UstepperS32.h>

#include <Wire.h>
#include "uSerial.h"

#define ADR 0x10
#define MAX_BUFFER 4  // Bytes
#define RFLAGS_SIZE 1



UstepperS32 stepper;
static uint8_t state = 0x00;

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
        int8_t v;
        readValue<int8_t>(v, rx_buf, rx_length);
        stepper.enableStallguard(v, true, 2);
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

    case ENABLECLOSEDLOOP:
      Serial.print("Executing ENABLECLOSEDLOOP\n");
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


    default:
      Serial.print("Unknown command\n");
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


      // case GETMOTORSTATE:
      //   {
      //     Serial.print("Executing GETMOTORSTATE\n");
      //     writeValue<uint8_t>(stepper.driver.readMotorStatus(), tx_buf, tx_length);
      //     tx_data_ready = 1;
      //     break;
      //   }


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
        // writeValue<uint8_t>(!stepper.getMotorState(STALLGUARD2), tx_buf, tx_length);
        // writeValue<uint8_t>(stepper.driver.readRegister(DRV_STATUS) & (1 << 24) ? 1 : 0, tx_buf, tx_length);

        // Serial.println(stepper.driver.readRegister(DRV_STATUS) & (1 << 24), HEX);
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
      Serial.print("Unknown function\n");

      // Instead of sendinf a zero buffer, set the tx_length to 0 to only send return flags
      // writeValue<uint32_t>(0, tx_buf, tx_length); 
      tx_length = 0;
      break;
  }
}

uint8_t stepper_state_flags(){
  state = stepper.isStalled();
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
  // stepper.enableStallguard(10, true, 60);
  // stepper.disableStallguard();

  // stepper.encoder.encoderStallDetectSensitivity = 1;  //Encoder stalldetect sensitivity - From -10 to 1 where lower number is less sensitive and higher is more sensitive. -0.25 works for most.
  // stepper.encoder.encoderStallDetectEnable = 1;           //Enable the encoder stall detect

  Serial.begin(9600);

  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop(void) {

  state |= stepper.isStalled() << 0 ;

  if (rx_data_ready) {
    rx_data_ready = 0;
    stepper_receive_handler(reg);
  }


  // if (!stepper.getMotorState(STALLGUARD2)) {
  //   Serial.println("STALLED");
  // }
  delay(1);
}
