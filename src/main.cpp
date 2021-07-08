#include "SCD30_MB.hpp"
#include "K30_MB.h"
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

#define FILENAME_PREFIX "/measurements_"
#define FILENAME_POSTFIX ".csv"
std::string filename = "";

std::string timestamp() {
  char buf[128]{0};
  time_t t;
  time(&t);
  strftime(buf, 128, "%FT%Hh%Mm%S", gmtime(&t));
  return std::string{buf};
}

const std::string delim = ","; // Csv column delimiter
const std::string sep = ".";   // Decimal seperator
const auto header = "time" + delim + "variable" + delim + "value";
std::stringstream& fmt_meas(std::stringstream& ss, std::string variable,
                            std::string value) {
  ss << timestamp() << delim << variable << delim << value << '\n';
  return ss;
}
std::stringstream& fmt_meas(std::stringstream& ss, std::string variable,
                            float value, int precision = 8) {
  ss << timestamp() << delim << variable << delim << std::setprecision(precision)
     << value << '\n';
  return ss;
}

SCD30_MB scd30_1;
SCD30_MB scd30_2;
K30_MB k30fr;
K30_MB k33elg;


Adafruit_BME280 bme280{};
auto bme_temp = bme280.getTemperatureSensor();
auto bme_pres = bme280.getPressureSensor();
auto bme_hume = bme280.getHumiditySensor();

inline bool log_fail(const char* msg, bool val, bool freeze = true) {
  if (val) {
    log_i("%s SUCCESS", msg);
  } else {
    log_e("%s FAILURE                <--- WARNING", msg);
    if (freeze) {
      while (true)
        delay(1000);
    }
  }
  return val;
}

std::string uid(int len = 6) {
  static char al[] = "abcdefghijklmnopqrstuvxyz";
  std::stringstream ss{""};
  for (int i = 0; i < len; i++) {
    ss << al[rand() % 26];
  }
  return ss.str();
}

void setup() {
  Serial.begin(115200);

  log_i("Initializing sensors");
  // Set up CO2 Sensors
  Serial1.begin(19200, SERIAL_8N1, U1_RX, U1_TX);
  Serial2.begin(9600, SERIAL_8N1, U2_RX, U2_TX); //Hardware serial port 2

  static auto mb1 = Modbus(&Serial1);
  static auto mb2 = Modbus(&Serial2);
  k30fr = K30_MB(&mb2, 0x68);   // Constructor using modbus 2 and address 0x68
  k33elg = K30_MB(&mb2, 0x69);  // Constructor using modbus 2 and address 0x69 (nice)
  scd30 = SCD30_MB(&mb1);
  log_fail("SCD30 initialization", scd30.sensor_connected());
  log_fail("K30-FR initialization", k30fr.sensor_connected());
  log_fail("K33-ELG initialization", k33elg.sensor_connected());

  // Set up I2C peripherals
  log_fail("I2C initialization", Wire.begin(I2C_SDA, I2C_SCL));
  log_fail("BME280 Initialization", bme280.begin(0x76, &Wire));

  // Set up SD card
  SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI);
  log_fail("SD initialization", SD.begin(SD_CSN, SPI));

  // Configure SCD30s
  pinMode(SENS1_NRDY, INPUT);
  digitalWrite(SENS1_EN, HIGH);
  scd30.set_meas_interval(2);
  scd30.start_cont_measurements(0x0000);

  // Create new file for measurements
  filename = FILENAME_PREFIX + uid() + FILENAME_POSTFIX;
  log_i("Creating file %s", filename.c_str());
  auto file = SD.open(filename.c_str(), FILE_WRITE);
  file.println(header.c_str());
  file.close();
}

void loop() {
  std::stringstream ss{""};
  // Wait until all CO2 sensors have data.
  scd30.block_until_data_ready();

  // Read CO2 concenctration, temperature and humidity from SCD30
  SCD30_Measurement scd30_meas{};
  scd30.read_measurement(&scd30_meas);
  fmt_meas(ss, "scd30_co2", scd30_meas.co2);
  fmt_meas(ss, "scd30_temperature", scd30_meas.temperature);
  fmt_meas(ss, "scd30_humidity", scd30_meas.humidity_percent);

  // Read CO2 concenctration from K30-FR
  float k30fr_meas;
  k30fr.read_measurement(k30fr_meas);
  fmt_meas(ss, "K30_co2", k30fr_meas);

  // Read CO2 concentration from K33-ELG
  float k33elg_meas;
  k33elg.read_measurement(k33elg_meas);
  fmt_meas(ss, "K33_co2", k33elg_meas);

  // Read BME280 environmental data
  sensors_event_t temp, hume, pres;
  bme_temp->getEvent(&temp);
  bme_hume->getEvent(&hume);
  bme_pres->getEvent(&pres);
  fmt_meas(ss, "bme280_1_temperature", temp.temperature);
  fmt_meas(ss, "bme280_1_humidity", hume.relative_humidity);
  fmt_meas(ss, "bme280_1_pressure", pres.pressure, 9);

  // Log data for debugging
  log_d("%s", ss.str().c_str());

  // Save data to file
  auto file = SD.open(filename.c_str(), FILE_APPEND);
  if (!file) {
    log_e("Failed to open file %s", filename.c_str());
  } else {
    file.println(ss.str().c_str());
    file.flush();
  }
  file.close();
}