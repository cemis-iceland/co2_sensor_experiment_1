#include "K30_MB.h"
#include "modbus.h"
#include <Arduino.h>
#include <array>
#include <iomanip>
#include <sstream>
#include <stdint.h>

std::string bytes_to_str(const uint8_t* buf, size_t len) {
  std::stringstream ss{""};
  for (int i = 0; i < len; i++) {
    ss << std::hex << std::setw(2) << std::setfill('0') << (int)buf[i] << " ";
  }
  return ss.str();
}

K30_MB::K30_MB(Modbus* modbus) : mb{modbus} {}

bool K30_MB::sensor_connected() {
    static const auto req = 
        mb->create_request(ADDRESS, READ_INPUT, reg::READ_CO2, 1);
    uint8_t resp[1]{0};
    mb->send_request(req, resp, 1);
    return resp[0] == K30_MB::ADDRESS;
}

k30_err_t K30_MB::read_measurement(float* out) {
    const auto read_meas_request =
        mb->create_request(ADDRESS, READ_INPUT, reg::READ_CO2, 1);
    uint8_t response_buf[7]{0};

    mb->send_request(read_meas_request, response_buf, sizeof(response_buf));

    if (response_buf[1] != READ_INPUT) {
        log_e("Invalid response to read measurements request: ");
        log_e("%s", bytes_to_str(response_buf, 7).c_str());
        return k30_err_t::INVALID_RESPONSE;
    }

    uint16_t co2 = __bswap_16(*(uint16_t*)(response_buf + 3));
    
    *out = co2;

    return k30_err_t::OK;
}

k30_err_t K30_MB::disable_ABC() {
    const auto disable_abc_request =
        mb->create_request(ADDRESS, WRITE, reg::ABC_PERIOD, 0);
    uint8_t response_buf[8]{0};

    mb->send_request(disable_abc_request, response_buf, sizeof(response_buf));
    if (response_buf[1] != WRITE) {
        log_e("Invalid response to disable ABC request: ");
        log_e("%s", bytes_to_str(response_buf, 8).c_str());
        return k30_err_t::INVALID_RESPONSE;
    }

    return k30_err_t::OK;
}