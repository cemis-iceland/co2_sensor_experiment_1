#pragma once
#ifndef modbus_lib_cemis
#define modbus_lib_cemis 1
#include <Arduino.h>
#include <array>
#include <iomanip>
#include <sstream>
#include <stdint.h>

class Modbus {
public:
  Modbus(){};
  Modbus(Stream* serial) : serial(serial){};

  typedef std::array<uint8_t, 8> Request;
  enum class Fcode { READ = 0x03, WRITE = 0x06 };
  static Fcode READ, WRITE;
  Request create_request(uint8_t address, Fcode, uint16_t register_start,
                         uint16_t content);
  void send_request(Request, uint8_t* response_buffer, uint8_t res_len);

private:
  Stream* serial;
};

#endif