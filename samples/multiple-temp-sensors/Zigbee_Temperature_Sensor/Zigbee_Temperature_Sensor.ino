// Copyright 2024 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @brief This example demonstrates Zigbee temperature sensor.
 *
 * The example demonstrates how to use Zigbee library to create a end device temperature sensor.
 * The temperature sensor is a Zigbee end device, which is controlled by a Zigbee coordinator.
 *
 * Proper Zigbee mode must be selected in Tools->Zigbee mode
 * and also the correct partition scheme must be selected in Tools->Partition Scheme.
 *
 * Please check the README.md for instructions and more detailed description.
 *
 * Created by Jan Procházka (https://github.com/P-R-O-C-H-Y/)
 */

#include <Arduino.h>
#ifndef ZIGBEE_MODE_ZCZR
#error "Zigbee coordinator/router mode is not selected in Tools->Zigbee mode"
#endif

#include "Zigbee.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <OneWireESP32.h>

/* Zigbee temperature sensor configuration */
uint8_t button = BOOT_PIN;

// One entry per DS18B20. Copy the addresses printed at boot into this table once you
// know which sensor is which. An entry left at 0 is filled at boot with the next
// unassigned sensor found on the bus, in search() order.
struct Probe {
  const char *label;  // shown on the OLED
  uint64_t addr;      // DS18B20 64-bit ROM address
};

Probe probes[] = {
  {"1", 0x0},
  {"2", 0x0},
  {"3", 0x0},
  {"4", 0x0},
};

// Optional Time cluster variables
struct tm timeinfo;
struct tm *localTime;
int32_t timezone;

// One Temperature Measurement endpoint per probe: probes[i] -> endpoint 10 + i
ZigbeeTempSensor zbTemp1 = ZigbeeTempSensor(10);
ZigbeeTempSensor zbTemp2 = ZigbeeTempSensor(11);
ZigbeeTempSensor zbTemp3 = ZigbeeTempSensor(12);
ZigbeeTempSensor zbTemp4 = ZigbeeTempSensor(13);
ZigbeeTempSensor *zbTemps[] = {&zbTemp1, &zbTemp2, &zbTemp3, &zbTemp4};
static_assert(sizeof(zbTemps) / sizeof(*zbTemps) == sizeof(probes) / sizeof(*probes),
              "need one Zigbee endpoint per probe");
#define OLED_ADDR 0x3C
Adafruit_SH1106G display(128, 64, &Wire, -1);
bool displayOk = false;

/************************ Temp sensor *****************************/
static bool assigned(uint64_t addr) {
  for (uint8_t p = 0; p < (sizeof(probes)/sizeof(*probes)); p++) {
    if (probes[p].addr == addr) return true;
  }
  return false;
}

static void temp_sensor_value_update(void *arg) {
  OneWire32 ds(D2);
  uint64_t addr[(sizeof(probes)/sizeof(*probes))];
  uint8_t found = ds.search(addr, (sizeof(probes)/sizeof(*probes)));
  Serial.printf("Found %u DS18B20 sensor(s)\r\n", found);
  for (uint8_t i = 0; i < found; i++) {
    Serial.printf("  %u: 0x%llx\r\n", i, addr[i]);
  }

  // Fill table entries left at 0 with sensors not already in the table
  for (uint8_t p = 0; p < (sizeof(probes)/sizeof(*probes)); p++) {
    for (uint8_t i = 0; probes[p].addr == 0 && i < found; i++) {
      if (!assigned(addr[i])) probes[p].addr = addr[i];
    }
    Serial.printf("Probe %s -> endpoint %u -> 0x%llx\r\n", probes[p].label, 10 + p, probes[p].addr);
  }

  for (;;) {
    ds.request();                    // start conversion on all sensors
    delay(750);

    float temps[(sizeof(probes)/sizeof(*probes))];
    bool ok[(sizeof(probes)/sizeof(*probes))];
    for (uint8_t p = 0; p < (sizeof(probes)/sizeof(*probes)); p++) {
      ok[p] = probes[p].addr != 0 && ds.getTemp(probes[p].addr, temps[p]) == 0;
      if (ok[p]) {
        Serial.printf("Probe %s: %.2f°C\r\n", probes[p].label, temps[p]);
        zbTemps[p]->setTemperature(temps[p]);
      } else {
        Serial.printf("Probe %s: read error\r\n", probes[p].label);
      }
    }

    if (displayOk) {
      display.clearDisplay();
      display.setTextColor(SH110X_WHITE);
      display.setTextSize(2);
      for (uint8_t p = 0; p < (sizeof(probes)/sizeof(*probes)); p++) {
        display.setCursor(0, p * 16);
        if (ok[p]) display.printf("%s %.1fF", probes[p].label, temps[p] * 9.0 / 5.0 + 32.0);
        else       display.printf("%s --", probes[p].label);
      }
      display.display();
    }
    delay(1000);
  }
}

static void builtin_temp_sensor_value_update(void *arg) {
  for (;;) {
    // Read temperature sensor value
    float tsens_value = temperatureRead();
    Serial.printf("Updated temperature sensor value to %.2f°C\r\n", tsens_value);
    // Update temperature value in Temperature sensor EP
    zbTemp1.setTemperature(tsens_value);

    if (displayOk) {
      display.clearDisplay();
      display.setTextColor(SH110X_WHITE);
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("Chip temperature");
      display.setTextSize(2);
      display.setCursor(0, 24);
      display.printf("%.1f F", tsens_value * 9.0 / 5.0 + 32.0);
      display.display();
    }
    delay(1000);
  }
}

/********************* Arduino functions **************************/
void setup() {
  pinMode(3, OUTPUT);  digitalWrite(3, LOW);   // enable RF switch (active low)
  pinMode(14, OUTPUT); digitalWrite(14, LOW);  // LOW = onboard ceramic antenna

  Serial.begin(115200);

  Wire.begin();  // D4 = SDA, D5 = SCL on the XIAO
  displayOk = display.begin(OLED_ADDR, true);
  if (!displayOk) {
    Serial.println("SH1106 not found - check wiring/address");
  }

  // Init button switch
  pinMode(button, INPUT_PULLUP);

  for (uint8_t p = 0; p < (sizeof(probes)/sizeof(*probes)); p++) {
    // Optional: set Zigbee device name and model
    zbTemps[p]->setManufacturerAndModel("Espressif", "ZigbeeTempSensor");

    // Set minimum and maximum temperature measurement value (10-50°C is default range for chip temperature measurement)
    // zbTemps[p]->setMinMaxValue(10, 50);
    zbTemps[p]->setMinMaxValue(-55, 125);

    // Optional: Set default (initial) value for the temperature sensor to 10.0°C to match the minimum temperature measurement value
    zbTemps[p]->setDefaultValue(10.0);

    // Optional: Set tolerance for temperature measurement in °C (lowest possible value is 0.01°C)
    zbTemps[p]->setTolerance(0.5);
  }

  // Optional: Time cluster configuration (default params, as this device will revieve time from coordinator)
  // Only needed on one endpoint
  zbTemp1.addTimeCluster();

  // Add endpoints to Zigbee Core
  for (uint8_t p = 0; p < (sizeof(probes)/sizeof(*probes)); p++) {
    Zigbee.addEndpoint(zbTemps[p]);
  }

  Serial.println("Starting Zigbee...");
  // When all EPs are registered, start Zigbee in Router mode
  if (!Zigbee.begin(ZIGBEE_ROUTER)) {
    Serial.println("Zigbee failed to start!");
    Serial.println("Rebooting...");
    ESP.restart();
  } else {
    Serial.println("Zigbee started successfully!");
  }
  Serial.println("Connecting to network");
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(100);
  }
  Serial.println();

  // Optional: If time cluster is added, time can be read from the coordinator
  timeinfo = zbTemp1.getTime();
  timezone = zbTemp1.getTimezone();

  Serial.println("UTC time:");
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");

  time_t local = mktime(&timeinfo) + timezone;
  localTime = localtime(&local);

  Serial.println("Local time with timezone:");
  Serial.println(localTime, "%A, %B %d %Y %H:%M:%S");

  // Start Temperature sensor reading task
  xTaskCreate(temp_sensor_value_update, "temp_sensor_update", 4096, NULL, 10, NULL);

  // Set reporting interval for temperature measurement in seconds, must be called after Zigbee.begin()
  // min_interval and max_interval in seconds, delta (temp change in 0,1 °C)
  // if min = 1 and max = 0, reporting is sent only when temperature changes by delta
  // if min = 0 and max = 10, reporting is sent every 10 seconds or temperature changes by delta
  // if min = 0, max = 10 and delta = 0, reporting is sent every 10 seconds regardless of temperature change
  for (uint8_t p = 0; p < (sizeof(probes)/sizeof(*probes)); p++) {
    zbTemps[p]->setReporting(1, 0, 1);
  }
}

void loop() {
  // Checking button for factory reset
  if (digitalRead(button) == LOW) {  // Push button pressed
    // Key debounce handling
    delay(100);
    int startTime = millis();
    while (digitalRead(button) == LOW) {
      delay(50);
      if ((millis() - startTime) > 3000) {
        // If key pressed for more than 3secs, factory reset Zigbee and reboot
        Serial.println("Resetting Zigbee to factory and rebooting in 1s.");
        delay(1000);
        Zigbee.factoryReset();
      }
    }
    for (uint8_t p = 0; p < (sizeof(probes)/sizeof(*probes)); p++) {
      zbTemps[p]->reportTemperature();
    }
  }
  delay(100);
}
