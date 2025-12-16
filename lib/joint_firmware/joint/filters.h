/**
 * @file filters.h
 * @author sbstorz
 * @brief Helper classes for FIR and IIR filters
 * @version 0.1
 * @date 2025-05-27
 *
 * @copyright Copyright (c) 2025
 *
 * This file contains time series filters that can be used to filter data.
 *
 */
#include <stdlib.h>

class Lowpass {

protected:
  float K, Ts, tau, x;

public:

  /**
  * @brief A simple lowpass filter class
  * @param gain
  * @param sampleTime
  * @param timeconstant
  * 
  */
  Lowpass(float gain = 1, float sampleTime = 0.1, float timeconstant = 1.0) {
    this->K = gain;
    this->Ts = sampleTime;
    this->tau = timeconstant;
    x = 0.0;
  }

  float updateState(float u) {
    x = (1 - (Ts / tau)) * x + K * (Ts / tau) * u;
    return x;
  }

  void resetState(void){
    x = 0.0;
  }
};

class MovMax {

protected:
  unsigned int M = 200;  // Window Size

  float *cb_data;
  unsigned int cb_index;


public:

  MovMax(float windowSize)
    : M(windowSize), cb_index(0), cb_data(0) {

    cb_data = (float *)malloc(windowSize * sizeof(float));  // allocate memory for buffer
  }

  float updateState(float u) {

    cb_data[cb_index] = u;
    cb_index = (cb_index + 1) % M;


    float max = 0;
    for (size_t i = 0; i < M; i++) {
      if (cb_data[i] > max) {
        max = cb_data[i];
      }
    }

    return max;
  }
};
