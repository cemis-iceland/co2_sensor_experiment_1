#include "SCD30_MB.hpp"
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_u-blox_GNSS
#include "pin_assignments.h"
#include <Adafruit_BME280.h>
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <chrono>
#include <iomanip>
#include <mbcontroller.h>
#include <sstream>
#include <sys/time.h>

SCD30_MB sensor1;
Adafruit_BME280 bme280{};
auto bme_temp = bme280.getTemperatureSensor();
auto bme_pres = bme280.getPressureSensor();
auto bme_hume = bme280.getHumiditySensor();

SFE_UBLOX_GPS gps_module;

#define FILENAME "/measurements.csv"

struct Measurement {
  SCD30_Measurement scd30;
  time_t time;
  float temperature;
  float humidity;
  float pressure;
  static char* header;
  std::string to_string() {
    std::ostringstream ss{""};
    ss << std::put_time(localtime(&time), "%FT%T") << ", " << scd30.co2 << ", "
       << scd30.temperature << ", " << scd30.humidity_percent << ", "
       << temperature << ", " << humidity << ", " << pressure;
    return ss.str();
  }
};
char* Measurement::header = "time, ppm, deg C, RH, deg C, RH, hPa";

inline void log_fail(const char* msg, bool val) {
  if (val) {
    log_e("%s SUCCESS", msg);
  } else {
    log_e("%s FAILURE", msg);
    for (;;)
      ;
  }
}

void setup() {
  Serial.begin(115200);
  ESP_LOGE("setup", "Initializing sensors");

  sensor1 = SCD30_MB{&Serial1, U2_RX, U2_TX};
  ESP_LOGE("setup", "Initialized sc30, initializing BME280");

  log_fail("Wire initialization", Wire.begin(I2C_SDA, I2C_SCL));
  log_fail("BME280 Initialization", bme280.begin(0x76, &Wire));

  ESP_LOGE("setup", "Initializing SD card");
  SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI);
  log_fail("SD initialization", SD.begin(SD_CSN, SPI));
  if (!SD.exists(FILENAME)) {
    auto file = SD.open(FILENAME, FILE_WRITE);
    file.println(Measurement::header);
  }

  log_fail("GPS Initialization", gps_module.begin(Wire));
  gps_module.setI2COutput(COM_TYPE_UBX); // Set the I2C port to output UBX only
  gps_module.saveConfiguration(); // Save the current settings to flash and BBR

  // Set up SCD30
  pinMode(SENS1_NRDY, INPUT);
  digitalWrite(SENS1_EN, HIGH);
  sensor1.start_cont_measurements(0x0000);
  sensor1.set_meas_interval(2);

  // Await time signal from GPS
  while (!(gps_module.getTimeValid() && gps_module.getDateValid())) {
    log_e("Waiting for GPS time");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
  log_e("Gps time ready");
  // Surely this isn't the best way to get a timestamp
  tm t;
  std::stringstream ss{""};
  ss << gps_module.getYear() << gps_module.getMonth() << gps_module.getDay()
     << gps_module.getHour() << gps_module.getMinute()
     << gps_module.getSecond();
  ss >> std::get_time(&t, "%Y%m%d%H%M&S");
  time_t timestamp = mktime(&t);
  timeval tv = {timestamp, gps_module.getNanosecond() / 1000};
  settimeofday(&tv, {});
}

void loop() {
  SCD30_Measurement meas{};
  sensor1.read_measurement_blocking(&meas, 3000, 500);
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

  auto file = SD.open("/measurements.csv", FILE_APPEND);
  if (!file) {
    log_e("Failed to open file");
  } else {
    file.println(data.to_string().c_str());
    file.flush();
  }
  file.close();
}
