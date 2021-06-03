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

SCD30_MB::SCD30_MB(ISerial* serial, uint8_t rx_pin, uint8_t tx_pin) {
  this->serial = serial;
  this->serial->begin(19200, SERIAL_8N1, rx_pin, tx_pin);
}

scd30_err_t SCD30_MB::data_ready(bool* out) {
  Request request{ADDRESS, READ, reg::DATA_READY, 0x0001, 0x3DA1};
  uint8_t response_buf[7]{0};

  if (serial->available()) {
    while (serial->read() != -1)
      ;
  };
  serial->write(reinterpret_cast<uint8_t*>(&request), sizeof(request));

  vTaskDelay(10 * portTICK_PERIOD_MS);

  serial->readBytes(response_buf, 7);
  serial->flush();

  if (response_buf[1] != 0x03) {
    log_e("Invalid response to data ready request: ");
    log_e("%s", bytes_to_str(response_buf, 7).c_str());
    return scd30_err_t::INVALID_RESPONSE;
  }
  *out = (bool)response_buf[4];
  return scd30_err_t::OK;
}

scd30_err_t SCD30_MB::read_measurement(SCD30_Measurement* out) {
  Request read_meas_request{ADDRESS, READ, reg::READ_DATA, 0x0006, 0x4C60};
  uint8_t response_buf[17]{0};
  if (serial->available()) {
    while (serial->read() != -1)
      ;
  };
  serial->write(reinterpret_cast<uint8_t*>(&read_meas_request),
                sizeof(read_meas_request));
  vTaskDelay(10 * portTICK_PERIOD_MS);
  serial->readBytes(response_buf, 17);

  if (response_buf[1] != 0x03) {
    log_e("Invalid response to read measurements request: ");
    log_e("%s", bytes_to_str(response_buf, 17).c_str());
    return scd30_err_t::INVALID_RESPONSE;
  }

  const auto rb{response_buf};
  uint32_t co2_t = (uint32_t)rb[3 + 3] << 0 * 8 | (uint32_t)rb[3 + 2] << 1 * 8 |
                   (uint32_t)rb[3 + 1] << 2 * 8 | (uint32_t)rb[3 + 0] << 3 * 8;
  uint32_t temp_t = (uint32_t)rb[7 + 3] << 0 * 8 |
                    (uint32_t)rb[7 + 2] << 1 * 8 |
                    (uint32_t)rb[7 + 1] << 2 * 8 | (uint32_t)rb[7 + 0] << 3 * 8;
  uint32_t humid_t =
      (uint32_t)rb[11 + 3] << 0 * 8 | (uint32_t)rb[11 + 2] << 1 * 8 |
      (uint32_t)rb[11 + 1] << 2 * 8 | (uint32_t)rb[11 + 0] << 3 * 8;

  out->co2 = *reinterpret_cast<float*>(&co2_t);
  out->temperature = *reinterpret_cast<float*>(&temp_t);
  out->humidity_percent = *reinterpret_cast<float*>(&humid_t);

  return scd30_err_t::OK;
}
