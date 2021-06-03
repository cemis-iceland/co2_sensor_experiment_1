#include "SCD30_MB.hpp"
#include <Arduino.h>
#include <array>
#include <iomanip>
#include <sstream>
#include <stdint.h>

#include "SCD30_MB.hpp"

std::string bytes_to_str(uint8_t* buf, size_t len) {
  std::stringstream ss{""};
  for (int i = 0; i < len; i++) {
    ss << std::hex << std::setw(2) << std::setfill('0') << (int)buf[i] << " ";
  }
  ss << std::endl;
  return ss.str();
}

void clear_buffer(ISerial* serial) {
  if (serial->available()) {
    while (serial->read() != -1) {
    };
  };
}

SCD30_MB::SCD30_MB(ISerial* serial, uint8_t rx_pin, uint8_t tx_pin) {
  this->serial = serial;
  this->serial->begin(19200, SERIAL_8N1, rx_pin, tx_pin);
}

/// Ask the sensor whether it has a new measurement ready
/// @param out pointer to bool where result will be stored.
scd30_err_t SCD30_MB::data_ready(bool* out) {
  Request request{ADDRESS, READ, reg::DATA_READY, 0x0001, 0x3DA1};
  uint8_t response_buf[7]{0};

  clear_buffer(serial);
  serial->write(request, sizeof(request));
  vTaskDelay(10 / portTICK_PERIOD_MS);
  serial->readBytes(response_buf, 7);

  if (response_buf[1] != 0x03) {
    log_e("Invalid response to data ready request: ");
    log_e("%s", bytes_to_str(response_buf, 7).c_str());
    return scd30_err_t::INVALID_RESPONSE;
  }
  *out = (bool)response_buf[4];
  return scd30_err_t::OK;
}

/// Ask the sensor for the latest measured values and store them in out
scd30_err_t SCD30_MB::read_measurement(SCD30_Measurement* out) {
  Request read_meas_request{ADDRESS, READ, reg::READ_DATA, 0x0006, 0x4C60};
  uint8_t response_buf[17]{0};

  clear_buffer(serial);
  serial->write(read_meas_request, sizeof(read_meas_request));
  vTaskDelay(10 / portTICK_PERIOD_MS);
  serial->readBytes(response_buf, 17);

  if (response_buf[1] != 0x03) {
    log_e("Invalid response to read measurements request: ");
    log_e("%s", bytes_to_str(response_buf, 17).c_str());
    return scd30_err_t::INVALID_RESPONSE;
  }

  uint32_t co2 = __bswap_32(*(uint32_t*)(response_buf + 3));
  uint32_t temperature = __bswap_32(*(uint32_t*)(response_buf + 7));
  uint32_t humidity = __bswap_32(*(uint32_t*)(response_buf + 11));
  // The below code type puns to float and outputs the vales immediately.
  memcpy(&out->co2, &co2, sizeof(co2));
  memcpy(&out->temperature, &temperature, sizeof(temperature));
  memcpy(&out->humidity_percent, &humidity, sizeof(humidity));

  return scd30_err_t::OK;
}
