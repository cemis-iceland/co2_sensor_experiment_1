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

bool sensor1_present, sensor2_present;
SCD30_MB sensor1;
SCD30_MB sensor2;

bool bme280_present;
Adafruit_BME280 bme280{};
auto bme_temp = bme280.getTemperatureSensor();
auto bme_pres = bme280.getPressureSensor();
auto bme_hume = bme280.getHumiditySensor();

SFE_UBLOX_GPS gps_module;

#define FILENAME_PREFIX "/measurements_"
#define FILENAME_POSTFIX ".csv"
std::string filename = "";

struct Measurement {
  SCD30_Measurement sens1{0};
  SCD30_Measurement sens2{0};
  std::string gps_time = "";
  int32_t latitude = 0;   // in degrees * 10^-7
  int32_t longditude = 0; // --||--
  int32_t altitude = 0;   // above global MSL in mm
  float temperature = 0;
  float humidity = 0;
  float pressure = 0;
  static char* header;
  std::string to_string() {
    const auto delim = ",";
    std::ostringstream ss{""};
    ss << gps_time << delim;
    ss << std::setprecision(9) << ((double)latitude) / 1e7 << delim;
    ss << std::setprecision(9) << ((double)longditude) / 1e7 << delim;
    ss << std::setprecision(5) << ((double)altitude) / 1e3 << delim;
    ss << std::setprecision(4) << sens1.co2 << delim;
    ss << sens1.temperature << delim;
    ss << sens1.humidity_percent << delim;
    ss << sens2.co2 << delim;
    ss << sens2.temperature << delim;
    ss << sens2.humidity_percent << delim;
    ss << temperature << delim;
    ss << humidity << delim;
    ss << pressure;
    return ss.str();
  }
};

char* Measurement::header =
    "Datetime, Latitude (deg), Longditude (deg), Altitude MSL (m), "
    "S1_CO2 (ppm), S1_Temperature (C), S1_Humidity (%), S2_CO2 (ppm), "
    "S2_Temperature (C), S2_Humidity (%), BME_Temperature (C), "
    "BME_Humidity (%), BME_Pressure (hPa)";

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

std::string get_gps_time(SFE_UBLOX_GPS& gps = gps_module) {
  std::stringstream ss{""};
  ss << (int)gps_module.getYear() << "-" << std::setw(2) << std::setfill('0')
     << (int)gps_module.getMonth() << "-" << std::setw(2) << std::setfill('0')
     << (int)gps_module.getDay() << "T" << std::setw(2) << std::setfill('0')
     << (int)gps_module.getHour() << "h" << std::setw(2) << std::setfill('0')
     << (int)gps_module.getMinute() << "m" << std::setw(2) << std::setfill('0')
     << (int)gps_module.getSecond();
  return ss.str();
}

void setup() {
  Serial.begin(115200);

  log_i("Initializing sensors");
  // Set up CO2 Sensors
  sensor1 = SCD30_MB(&Serial1, U1_RX, U1_TX);
  sensor2 = SCD30_MB{&Serial2, U2_RX, U2_TX};
  sensor1_present =
      log_fail("Sensor 1 initialization", sensor1.sensor_connected(), false);
  sensor2_present =
      log_fail("Sensor 2 initialization", sensor2.sensor_connected(), false);

  // Set up I2C peripherals
  log_fail("Wire initialization", Wire.begin(I2C_SDA, I2C_SCL));
  log_fail("GPS Initialization", gps_module.begin(Wire));
  bme280_present =
      log_fail("BME280 Initialization", bme280.begin(0x76, &Wire), false);

  // Set up SD card
  SPI.begin(SPI_SCLK, SPI_MISO, SPI_MOSI);
  log_fail("SD initialization", SD.begin(SD_CSN, SPI));

  // Configure gps
  gps_module.setI2COutput(COM_TYPE_UBX); // Set the I2C port to output UBX only
  gps_module.saveConfiguration(); // Save the current settings to flash and BBR

  // Configure SCD30s
  pinMode(SENS1_NRDY, INPUT);
  pinMode(SENS2_NRDY, INPUT);
  digitalWrite(SENS1_EN, HIGH);
  digitalWrite(SENS2_EN, HIGH);
  sensor1.set_meas_interval(2);
  sensor2.set_meas_interval(2);
  sensor1.start_cont_measurements(0x0000);
  sensor2.start_cont_measurements(0x0000);

  // Await time signal from GPS
  while (!(gps_module.getTimeValid() && gps_module.getDateValid())) {
    log_i("Waiting for GPS time...");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
  log_i("Gps time ready");

  // Create new file for measurements
  filename = FILENAME_PREFIX + get_gps_time() + FILENAME_POSTFIX;
  log_i("Creating file %s", filename.c_str());
  auto file = SD.open(filename.c_str(), FILE_WRITE);
  file.println(Measurement::header);
  file.close();
}

void loop() {
  Measurement data{};
  // Wait until both CO2 sensors have data.
  if (sensor1_present) {
    sensor1.block_until_data_ready();
  }
  if (sensor2_present) {
    sensor2.block_until_data_ready();
  }

  // Read CO2 concenctrations
  if (sensor1_present)
    sensor1.read_measurement(&data.sens1);
  if (sensor2_present)
    sensor2.read_measurement(&data.sens2);

  // Read BME280 environmental data
  if (bme280_present) {
    sensors_event_t temp, hume, pres;
    bme_temp->getEvent(&temp);
    bme_hume->getEvent(&hume);
    bme_pres->getEvent(&pres);
    data.temperature = temp.temperature;
    data.humidity = hume.relative_humidity;
    data.pressure = pres.pressure;
  }

  // Read time and position data
  data.gps_time = get_gps_time();
  data.latitude = gps_module.getLatitude();
  data.longditude = gps_module.getLongitude();
  data.altitude = gps_module.getAltitudeMSL();

  // Log data for debugging
  log_d("%s", data.to_string().c_str());

  // Save data to file
  auto file = SD.open(filename.c_str(), FILE_APPEND);
  if (!file) {
    log_e("Failed to open file %s", filename.c_str());
  } else {
    file.println(data.to_string().c_str());
    file.flush();
  }
  file.close();
}
