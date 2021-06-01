#include "pin_assignments.h"
#include <Arduino.h>
#include <mbcontroller.h>

void setup() {
  Serial.begin(115200);
  ESP_LOGE("Wow");
  Serial1.begin(19200, SERIAL_8N1, U1_RX, U1_TX);

  delay(500);
}

void loop() {}