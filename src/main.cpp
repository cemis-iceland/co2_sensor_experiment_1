#include "SCD30_MB.hpp"
#include "pin_assignments.h"
#include <Adafruit_BME280.h>
#include <Arduino.h>
#include <iomanip>
#include <mbcontroller.h>
#include <sstream>

SCD30_MB sensor1;
Adafruit_BME280 bme280{};
auto bme_temp = bme280.getTemperatureSensor();
auto bme_pres = bme280.getPressureSensor();
auto bme_hume = bme280.getHumiditySensor();

struct Measurement {
  SCD30_Measurement scd30;
  time_t time;
  float temperature;
  float humidity;
  float pressure;
  std::string to_string() {
    std::ostringstream ss{""};
    ss << "co2: {" << scd30.co2 << "ppm, " << scd30.temperature << "°C, "
       << scd30.humidity_percent << "%"
       << "}, bme: {" << temperature << "°C, " << humidity << "%, " << pressure
       << "hPa}";
    return ss.str();
  }
};

void setup() {
  Serial.begin(115200);
  ESP_LOGE("setup", "Initializing sensors");

  sensor1 = SCD30_MB{&Serial1, U1_RX, U1_TX};
  ESP_LOGE("setup", "Initialized sc30, initializing BME280");
  Wire.begin(I2C_SDA, I2C_SCL);
  bme280.begin(0x76, &Wire);
}

void loop() {
  SCD30_Measurement meas{};
  sensor1.read_measurement_blocking(&meas);
  sensors_event_t temp, hume, pres;
  bme_temp->getEvent(&temp);
  bme_hume->getEvent(&hume);
  bme_pres->getEvent(&pres);

  Measurement data{};
  data.scd30 = meas;
  time(&data.time);
  data.temperature = temp.temperature;
  data.humidity = hume.relative_humidity;
  data.pressure = pres.pressure;
  log_e("%s", data.to_string().c_str());
}