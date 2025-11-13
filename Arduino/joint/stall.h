/**
 * @file stall.h
 * @author Sebastian Storz
 * @brief Helper functions for improved stall detection
 * @version 0.1
 * @date 2025-05-27
 *
 * @copyright Copyright (c) 2025
 *
 *
 */
#include <stdlib.h>

float stall_threshold(float qd_set, float offset){
  /* y = ax + b */
  float a = STALL_SLOPE;
  float b = offset;
  if(qd_set >= STALL_WINDOW_B1 && qd_set <= STALL_WINDOW_B2){
    b += STALL_WINDOW_OFFSET;
  }
  return a*qd_set+b;
}
