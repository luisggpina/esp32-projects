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
 * Hot tub controller for a Seeed XIAO ESP32-C6, running as a Zigbee router
 * paired with Hubitat (see hubitat_driver.groovy for the parent driver).
 *
 * Based off the sample created by Jan Procházka (https://github.com/P-R-O-C-H-Y/)
 *
 * Modified from Espressif's Zigbee_Temperature_Sensor example to read four
 * DS18B20 probes, switch three relays and show the readings on an OLED.
 *
 * Zigbee endpoints:
 *   10-13  Temperature Measurement, one per entry in probes[]
 *   14-16  On/Off for the big pump, circulation pump and heater (relays[])
 *
 * Wiring:
 *   D2     1-Wire bus for all four DS18B20s, 4.7k pull-up to 3V3
 *   D4/D5  SH1106 OLED (I2C SDA/SCL)
 *   D7     heater relay
 *   D8     circulation pump relay
 *   D9     big pump relay
 *   BOOT   short press: report all temperatures now;
 *          hold 3 s: factory-reset Zigbee and leave the network
 *
 * Build with Zigbee mode ZCZR (coordinator/router) and the zigbee_zczr
 * partition scheme; the sketch refuses to compile in any other Zigbee mode.
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

#include <esp_task_wdt.h>

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

// Latest probe readings, shared between tasks.
//
// snapQ is a FreeRTOS queue of length 1 used as an atomic register (FreeRTOS calls
// this a "mailbox"): the sensor task is the only writer, any task may read.
//   - Write: xQueueOverwrite() replaces the single slot; it never blocks and never
//     queues up old values. Only valid on a queue of length 1.
//   - Read:  xQueuePeek() copies the slot out without removing it, so every reader
//     sees the latest snapshot.
// Both copy the whole struct inside the queue's critical section, so a reader gets
// either the old snapshot or the new one, never a mix of the two.
// Until the first write, xQueuePeek() fails: treat that as "no data", i.e. every
// probe is bad. The queue's memory is allocated once, in setup(), and never grows.
// Do not call these from an ISR (the ...FromISR variants exist for that).
struct Snapshot {
  float    temp[sizeof(probes) / sizeof(*probes)];  // °C, last good read
  bool     ok[sizeof(probes) / sizeof(*probes)];    // did the last read succeed?
  uint32_t at[sizeof(probes) / sizeof(*probes)];    // millis() of last good read;
                                                    // age = millis() - at, wrap-safe
};
QueueHandle_t snapQ;

// One Temperature Measurement endpoint per probe: probes[i] -> endpoint 10 + i
ZigbeeTempSensor zbTemp1 = ZigbeeTempSensor(10);
ZigbeeTempSensor zbTemp2 = ZigbeeTempSensor(11);
ZigbeeTempSensor zbTemp3 = ZigbeeTempSensor(12);
ZigbeeTempSensor zbTemp4 = ZigbeeTempSensor(13);
ZigbeeTempSensor *zbTemps[] = {&zbTemp1, &zbTemp2, &zbTemp3, &zbTemp4};
static_assert(sizeof(zbTemps) / sizeof(*zbTemps) == sizeof(probes) / sizeof(*probes),
              "need one Zigbee endpoint per probe");

// Relays: one On/Off endpoint per load, endpoints 14-16 (relays[i] -> endpoint 14 + i).
// The pins drive the 4-channel opto-isolated DIN relay board, jumpered for high-level
// trigger, so HIGH = on, and the relays stay off while the ESP32 is unpowered or
// resetting. Keep the order in sync with RELAY_NAMES in hubitat_driver.groovy.
ZigbeePowerOutlet zbRelay1 = ZigbeePowerOutlet(14);
ZigbeePowerOutlet zbRelay2 = ZigbeePowerOutlet(15);
ZigbeePowerOutlet zbRelay3 = ZigbeePowerOutlet(16);

struct Relay {
  const char *name;
  uint8_t pin;
  ZigbeePowerOutlet *ep;
};

Relay relays[] = {
  {"Big pump",  D9, &zbRelay1},  // endpoint 14
  {"Circ pump", D8, &zbRelay2},  // endpoint 15
  {"Heater",    D7, &zbRelay3},  // endpoint 16
};

static void setRelay(uint8_t r, bool on) {
  digitalWrite(relays[r].pin, on ? HIGH : LOW);
  Serial.printf("%s (endpoint %u) %s\r\n", relays[r].name, 14 + r, on ? "on" : "off");
}

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

// Safety checks. Runs in its own task, started before the Zigbee wait in
// setup(), so it keeps running when the hub is unreachable.
static void safety_task(void *arg) {
  // Subscribe to the task watchdog. The core's sdkconfig sets a 5 s timeout and a
  // panic on expiry, so if this task stops feeding it, the board resets and the
  // relays fail off.

  if (esp_task_wdt_add(NULL) != ESP_OK) {
    Serial.println("safety: could not subscribe to the task watchdog");
  }
  // Overflows every 136 years, to 0, benign
  for (uint32_t pass = 0;; pass++) {
    esp_task_wdt_reset();
    if (pass % 5 == 0) {
      Snapshot s;
      if (xQueuePeek(snapQ, &s, 0) == pdTRUE) {
        for (uint8_t p = 0; p < (sizeof(probes)/sizeof(*probes)); p++) {
          Serial.printf("snap %s: %s %.2f°C, %lu ms old\r\n", probes[p].label,
                        s.ok[p] ? "ok " : "bad", s.temp[p], (unsigned long)(millis() - s.at[p]));
        }
      } else {
        Serial.println("snap: no data yet");
      }
    }
    delay(1000);
  }
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

  Snapshot s = {};
  for (;;) {
    ds.request();                    // start conversion on all sensors
    delay(750);

    for (uint8_t p = 0; p < (sizeof(probes)/sizeof(*probes)); p++) {
      float t;
      s.ok[p] = probes[p].addr != 0 && ds.getTemp(probes[p].addr, t) == 0;
      if (s.ok[p]) {
        s.temp[p] = t;
        s.at[p] = millis();
        Serial.printf("Probe %s: %.2f°C\r\n", probes[p].label, t);
        zbTemps[p]->setTemperature(t);
      } else {
        Serial.printf("Probe %s: read error\r\n", probes[p].label);
      }
    }

    xQueueOverwrite(snapQ, &s);  // publish this cycle's snapshot to all readers

    if (displayOk) {
      display.clearDisplay();
      display.setTextColor(SH110X_WHITE);
      display.setTextSize(2);
      for (uint8_t p = 0; p < (sizeof(probes)/sizeof(*probes)); p++) {
        display.setCursor(0, p * 16);
        if (s.ok[p]) display.printf("%s %.1fF", probes[p].label, s.temp[p] * 9.0 / 5.0 + 32.0);
        else       display.printf("%s --", probes[p].label);
      }
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

  // Relays start off
  for (uint8_t r = 0; r < (sizeof(relays)/sizeof(*relays)); r++) {
    pinMode(relays[r].pin, OUTPUT);
    digitalWrite(relays[r].pin, LOW);
  }
  zbRelay1.onPowerOutletChange([](bool on) { setRelay(0, on); });
  zbRelay2.onPowerOutletChange([](bool on) { setRelay(1, on); });
  zbRelay3.onPowerOutletChange([](bool on) { setRelay(2, on); });

  for (uint8_t p = 0; p < (sizeof(probes)/sizeof(*probes)); p++) {
    zbTemps[p]->setManufacturerAndModel("Espressif", "ZigbeeTempSensor");
    zbTemps[p]->setMinMaxValue(-55, 125);  // DS18B20 measuring range, °C
    zbTemps[p]->setTolerance(0.5);         // DS18B20 accuracy, ±0.5 °C
    // Value the endpoint holds until its probe's first good read (not a real reading)
    zbTemps[p]->setDefaultValue(10.0);
  }

  // Add endpoints to Zigbee Core
  for (uint8_t p = 0; p < (sizeof(probes)/sizeof(*probes)); p++) {
    Zigbee.addEndpoint(zbTemps[p]);
  }
  for (uint8_t r = 0; r < (sizeof(relays)/sizeof(*relays)); r++) {
    relays[r].ep->setManufacturerAndModel("Espressif", "ZigbeeTempSensor");
    Zigbee.addEndpoint(relays[r].ep);
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

  // One-slot mailbox for Snapshot; see the comment at struct Snapshot.
  snapQ = xQueueCreate(1, sizeof(Snapshot));
  if (!snapQ) { Serial.println("snapshot queue alloc failed"); ESP.restart(); }

  // Start Temperature sensor reading task
  xTaskCreate(temp_sensor_value_update, "temp_sensor_update", 4096, NULL, 10, NULL);

  xTaskCreate(safety_task, "safety", 4096, NULL, 11, NULL);

  Serial.println("Connecting to network");
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(100);
  }
  Serial.println();

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
