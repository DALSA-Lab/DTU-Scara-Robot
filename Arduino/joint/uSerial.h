#ifndef USERIAL_H
#define USERIAL_H
#include <Arduino.h>

#define ACK 'O'
#define NACK 'N'

#define DUMP_BUFFER(buffer, size) \
  { \
    Serial.print("Buffer dump: "); \
    for (size_t i = 0; i < size; i++) { \
      Serial.print(buffer[i], HEX); \
      Serial.print(" "); \
    } \
    Serial.println(); \
  }

enum stp_reg_t {
  PING = 0x0f,
  SETUP = 0x10,
  SETRPM = 0x11,
  GETDRIVERRPM = 0x12,
  MOVESTEPS = 0x13,
  MOVEANGLE = 0x14,
  MOVETOANGLE = 0x15,
  GETMOTORSTATE = 0x16,
  RUNCOTINOUS = 0x17,
  ANGLEMOVED = 0x18,
  SETCURRENT = 0x19,
  SETHOLDCURRENT = 0x1A,
  SETMAXACCELERATION = 0x1B,
  SETMAXDECELERATION = 0x1C,
  SETMAXVELOCITY = 0x1D,
  ENABLESTALLGUARD = 0x1E,
  DISABLESTALLGUARD = 0x1F,
  CLEARSTALL = 0x20,
  ISSTALLED = 0x21,
  SETBRAKEMODE = 0x22,
  ENABLEPID = 0x23,
  DISABLEPID = 0x24,
  ENABLECLOSEDLOOP = 0x25,
  DISABLECLOSEDLOOP = 0x26,
  SETCONTROLTHRESHOLD = 0x27,
  MOVETOEND = 0x28,
  STOP = 0x29,
  GETPIDERROR = 0x2A,
  CHECKORIENTATION = 0x2B,
  GETENCODERRPM = 0x2C,
  HOME = 0x2D
};

/**
 * Compute the two' complement checksum of the `buffer` according to SAE J1708
 * @param buffer Pointer to buffer to compute checksum off
 * @param length Length of the buffer
 * @return Two's complement checksum.
 */
uint8_t generateChecksum(const uint8_t *buffer, size_t length);

// /**
//  * Reads a value from Serial Buffer of the specified type `T` into `val`
//  * @param val Reference to output variable
//  * @param rxBuf Buffer to read value from
//  * @param rx_length Length of the buffer
//  * @return 0 On success, -1 on Timeout, -2 on CHK fail
//  */
// template<typename T>
// int readValue(T &val, uint8_t *rxBuf, int rx_length);

// /**
//  * Reads a value from Serial Buffer of the specified type `T` into `val`
//  * @param val Reference to output variable
//  * @param rxBuf Buffer to read value from
//  * @param rx_length Length of the buffer
//  * @return 0 On success, -1 on Timeout, -2 on CHK fail
//  */
// template<typename T>
// int writeValue(const T val, uint8_t *txBuf, int tx_length);




/**
 * Reads a value from Serial Buffer of the specified type `T` into `val`
 * @param val Reference to output variable
 * @param rxBuf Buffer to read value from
 * @param rx_length Length of the buffer
 */
template<typename T>
void readValue(T &val, uint8_t *rxBuf, size_t rx_length) {
  memcpy(&val, rxBuf, rx_length);
}

/**
 * Writes a value to the Serial output buffer using a intermediate buffer.
 * @param val Reference to input variable
 * @param txBuf pointer to tx buffer
 * @param tx_length Length of the buffer
 * @return 0 On success
 */
template<typename T>
int writeValue(const T val, uint8_t *txBuf, size_t &tx_length) {
  tx_length = sizeof(T);
  memcpy(txBuf, &val, tx_length);
  return 0;
}

#endif
