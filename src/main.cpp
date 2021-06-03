#include "SCD30_MB.hpp"
#include "pin_assignments.h"
#include <Arduino.h>
#include <iomanip>
#include <mbcontroller.h>
#include <sstream>

SCD30_MB sensor1;

void setup() {
  Serial.begin(115200);
  ESP_LOGE("setup", "Initializing sensor.");
  sensor1 = SCD30_MB{&Serial1, U1_RX, U1_TX};
}

void loop() {
  // ESP_LOGE("debug", "Checking data ready");
  bool ready = false;
  sensor1.data_ready(&ready);
  while (!ready) {
    vTaskDelay(100 * portTICK_PERIOD_MS);
    sensor1.data_ready(&ready);
  };
  // log_e("Ready state: %s\n", ready ? "true" : "false");
  SCD30_Measurement meas{};
  // log_e("Reading measurements");
  sensor1.read_measurement(&meas);
  log_e("Read co2: %f, temp: %f, humidity: %f", meas.co2, meas.temperature,
        meas.humidity_percent);
}