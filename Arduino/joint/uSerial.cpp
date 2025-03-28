#include "uSerial.h"


uint8_t generateChecksum(const uint8_t *buffer, size_t length) {
  uint32_t sum = 0;  // Use a larger type to avoid overflow

  // Sum all bytes in the buffer
  for (size_t i = 0; i < length; ++i) {
    sum += buffer[i];
  }

  // Calculate the two's complement
  return static_cast<uint8_t>(~sum + 1);
}

