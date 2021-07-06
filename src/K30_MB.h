// Senseair K30 library using serial (modbus)
#pragma once
#include "modbus.h"
#include <Arduino.h>
#include <array>
#include <byteswap.h>
#include <stdint.h>

enum class k30_err_t { OK = 0, INVALID_RESPONSE, TIMEOUT };

class K30_MB {
public:
  K30_MB(){};
  K30_MB(Modbus* modbus){};
  k30_err_t read_measurement(float* out);
  k30_err_t disable_ABC();

  bool sensor_connected();
private:
  Modbus* mb;

  static const uint8_t READ_INPUT = 0x04;   // Modbus function code
  static const uint8_t READ_HOLDING = 0x03; // Modbus function code
  static const uint8_t WRITE = 0x06;        // Modbus function code
  static const uint8_t ADDRESS = 0xFE;      // K30 sensor address (technically "all sensors")

  enum reg {  // Register map of the K30 sensor
    READ_CO2 = 0x0003,
    READ_STATUS = 0x0000,

    ACKNOWLEDGE = 0x0000,
    CALIBRATION = 0x0001,
    ABC_PERIOD = 0x001F,
  };
};