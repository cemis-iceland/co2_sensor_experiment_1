#include "pin_assignments.h"
#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <SparkFun_SCD30_Arduino_Library.h>
#include <Wire.h>
#include <esp_pm.h>

#define SCREEN_SDA 21
#define SCREEN_SCL 22
#define SCREEN_RST 16
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

TwoWire i2c{0};
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &i2c, SCREEN_RST);

Adafruit_BME280 bme280{};
Adafruit_Sensor* bme_temp = bme280.getTemperatureSensor();
Adafruit_Sensor* bme_humidity = bme280.getHumiditySensor();
Adafruit_Sensor* bme_pressure = bme280.getPressureSensor();

SCD30 sensor1{};

esp_pm_config_esp32_t config{};

void task_loop(void*);

void setup() {
  Serial.begin(115200);

  config.light_sleep_enable = true;
  config.max_freq_mhz = 80;
  esp_pm_configure(&config);

  i2c.begin(SCREEN_SDA, SCREEN_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }

  if (!bme280.begin(0b01110110, &i2c)) {
    Serial.print("Failed to initialize BME280, stopping.");
    for (;;)
      ;
  }

  if (!sensor1.begin(i2c)) {
    Serial.print("Failed to initialize SCD30, stopping.");
    for (;;)
      ;
  }

  xTaskCreatePinnedToCore(task_loop,   // Function to be called
                          "main loop", // Name of task
                          1024 * 32,   // Stack size in bytes
                          NULL,        // Parameter to pass function
                          1,           // Task priority (0-24)
                          NULL,        // Task handle
                          APP_CPU_NUM  // Task core
  );
}

void task_loop(void* parameter) {
  while (1) {
    display.clearDisplay();
    display.setCursor(0, 0);

    sensors_event_t temp_event, humidity_event, pressure_event;
    bme_temp->getEvent(&temp_event);
    bme_humidity->getEvent(&humidity_event);
    bme_pressure->getEvent(&pressure_event);

    while (!sensor1.dataAvailable()) {
      Serial.println("No data");
      vTaskDelay(100);
    }
    auto co2_ppm = sensor1.getCO2();

    display.setTextColor(WHITE);
    display.setTextSize(1);

    display.print(temp_event.temperature);
    display.print("C ");
    display.print(humidity_event.relative_humidity);
    display.print("% ");
    display.print(pressure_event.pressure);
    display.println("Q");

    display.print("CO2_1: ");
    display.print(co2_ppm);
    display.println(" ppm");

    display.display();
    vTaskDelay(20);
    esp_sleep_enable_timer_wakeup(2 * 1000 * 1000);
    esp_deep_sleep_start();
  }
}

void loop() { vTaskDelay(portMAX_DELAY); }