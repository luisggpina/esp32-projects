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
 *   17     On/Off for the screen (no relay)
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
#include <OneWireESP32.h>

#include <esp_task_wdt.h>
#include <atomic>

#include <Preferences.h>

#include <U8g2lib.h>

// SH1106 128x64 on I2C (D4 = SDA, D5 = SCL), full frame buffer
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

/* Zigbee temperature sensor configuration */
uint8_t button = BOOT_PIN;

// One entry per probe role. The addresses are assigned by probe setup and kept in
// flash (see loadProbes()); an entry left at 0 has no probe.
struct Probe {
  const char *label;  // shown on the OLED
  uint64_t addr;      // DS18B20 64-bit ROM address
};

// Probe roles, in setup order; probes[i] reports on endpoint 10 + i. The heater
// probe is whichever one is left after the other three are assigned.
Probe probes[] = {
  {"Outside", 0x0},
  {"Top",     0x0},
  {"Bottom",  0x0},
  {"Heater",  0x0},
};
enum : uint8_t { P_OUTSIDE, P_TOP, P_BOTTOM, P_HEATER };  // indices into probes[]

const float    SETUP_DELTA_C   = 2.0 / 1.8;  // 2 °F: a change this big picks the probe
const uint32_t SETUP_UNPLUG_MS = 5000;       // no good read this long = unplugged

// Probe setup: ask for each role in turn (outside, water top, water bottom). The
// first unassigned probe that changes by more than 2 °F, or stops answering for 5 s
// and then answers again, gets that role. The heater is the probe left over. The bus
// is searched again on every cycle, because a single search sometimes misses probes,
// and every probe ever found is kept. The current mapping keeps being read and
// published meanwhile, and the new one is only used, and saved, at the end.
struct Candidate {
  uint64_t addr;
  bool taken;      // already given a role
  bool haveBase;   // have a reading since this role's prompt
  bool wasGone;    // stopped answering for SETUP_UNPLUG_MS since the prompt
  float base;      // reading when this role's prompt appeared
  uint32_t lastOk; // millis() of the last good read
  const char *role; // role name once assigned (for the screen), else nullptr
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

// Set to make the sensor task run probe setup: at boot with no stored mapping,
// when a new probe appears on the bus, or after 5 "on" commands for the screen
// within 60 s.
std::atomic<bool> setupRequested{false};

// Set by the safety task, read by the display
std::atomic<bool> freezeActive{false};
std::atomic<bool> hotActive{false};

// The display task draws the status screen and owns the panel's power; the sensor
// task draws the setup screens. oledMutex keeps them from using the OLED at the
// same time, and setupActive tells the display task to leave the screen alone.
SemaphoreHandle_t oledMutex;
std::atomic<bool> setupActive{false};

// One Temperature Measurement endpoint per probe: probes[i] -> endpoint 10 + i
ZigbeeTempSensor zbTemp1 = ZigbeeTempSensor(10);
ZigbeeTempSensor zbTemp2 = ZigbeeTempSensor(11);
ZigbeeTempSensor zbTemp3 = ZigbeeTempSensor(12);
ZigbeeTempSensor zbTemp4 = ZigbeeTempSensor(13);
ZigbeeTempSensor *zbTemps[] = {&zbTemp1, &zbTemp2, &zbTemp3, &zbTemp4};
static_assert(sizeof(zbTemps) / sizeof(*zbTemps) == sizeof(probes) / sizeof(*probes),
              "need one Zigbee endpoint per probe");

// Outputs: one On/Off endpoint each, endpoints 14-17 (relays[i] -> endpoint 14 + i).
// The screen (17) has no relay. The pins drive the 4-channel opto-isolated DIN relay board, jumpered for high-level
// trigger, so HIGH = on, and the relays stay off while the ESP32 is unpowered or
// resetting. Keep the order in sync with RELAY_NAMES in hubitat_driver.groovy.
ZigbeePowerOutlet zbRelay1 = ZigbeePowerOutlet(14);
ZigbeePowerOutlet zbRelay2 = ZigbeePowerOutlet(15);
ZigbeePowerOutlet zbRelay3 = ZigbeePowerOutlet(16);
ZigbeePowerOutlet zbRelay4 = ZigbeePowerOutlet(17);  // screen on/off

struct Relay {
  const char *name;
  uint8_t pin;
  ZigbeePowerOutlet *ep;
  std::atomic<bool> on;  // current state; written only by the safety task, through setRelay()
};

const uint8_t NO_PIN = 0xFF;  // an output with no GPIO (the screen)

Relay relays[] = {
  {"Big pump",  D9, &zbRelay1},  // endpoint 14
  {"Circ pump", D8, &zbRelay2},  // endpoint 15
  {"Heater",    D7, &zbRelay3},  // endpoint 16
  {"Screen", NO_PIN, &zbRelay4, true},   // endpoint 17; starts on
};

enum : uint8_t { BIG_PUMP, CIRC_PUMP, HEATER, SCREEN };  // indices into relays[], same order

// Purge: if the big pump hasn't run for 4 h, run it for 30 s with the heater off,
// to keep it from sticking. The heater is left off afterwards. Any command from
// Hubitat ends a purge: the big pump goes off, and the command runs 1 s later.
const uint32_t PURGE_IDLE_MS = 4UL * 60 * 60 * 1000;
const uint32_t PURGE_GAP_MS  = 2000;   // heater off -> big pump on
const uint32_t PURGE_RUN_MS  = 30000;

// Freeze protection: below 35 °F outside, the circulation pump is forced on and
// Hubitat can't turn it off; above 36 °F it's released, and goes back to what
// Hubitat last asked for. With no good outside reading for 5 min, assume it's
// freezing.
const float    FREEZE_ON_C     = (35.0 - 32.0) / 1.8;
const float    FREEZE_OFF_C    = (36.0 - 32.0) / 1.8;
const uint32_t FREEZE_STALE_MS = 5UL * 60 * 1000;

// Over-temperature: at or above 110 °F at the water top, the big pump is turned off
// (no flow, so the heater stops), and Hubitat can't turn it on again until the top is
// below 108 °F. After that it stays off until Hubitat turns it on. With no good water
// top reading for 5 min, assume it's too hot.
const float    HOT_ON_C     = (110.0 - 32.0) / 1.8;
const float    HOT_OFF_C    = (108.0 - 32.0) / 1.8;
const uint32_t HOT_STALE_MS = 5UL * 60 * 1000;

bool wasConnected = false;        // to spot a (re)join

// Relay commands from Hubitat, passed from the Zigbee task to the safety task,
// which is the only task that switches relays.
struct RelayCmd { uint8_t relay; bool on; };
QueueHandle_t cmdQ;
TaskHandle_t safetyTask = NULL;

// Switch relay r and tell Hubitat. Only the safety task calls this, both for
// Hubitat's commands and for the board's own changes (purge, safety checks).
// It sets the endpoint's On/Off attribute itself and sends an On/Off command to the
// hub, because the library doesn't report the board's own changes (seen in
// testing). The command goes straight to the hub (coordinator 0x0000, endpoint 1),
// so it doesn't depend on bindings.
static void setRelay(uint8_t r, bool on) {
  if (relays[r].pin != NO_PIN) digitalWrite(relays[r].pin, on ? HIGH : LOW);
  relays[r].on = on;
  log_i("%s (endpoint %u) %s", relays[r].name, 14 + r, on ? "on" : "off");

  // Keep the endpoint's On/Off attribute in step with the relay, and tell Hubitat:
  // send an On/Off command for the new state from this relay's endpoint to the hub
  // (coordinator 0x0000, endpoint 1), the way a Zigbee wall switch would. The
  // driver turns it into the child's switch event. (Attribute reports for On/Off
  // never reach the hub from this stack.)
  esp_zb_zcl_on_off_cmd_t cmd;
  memset(&cmd, 0, sizeof(cmd));
  cmd.zcl_basic_cmd.src_endpoint = 14 + r;
  cmd.zcl_basic_cmd.dst_endpoint = 1;
  cmd.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
  cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
  cmd.on_off_cmd_id = on ? ESP_ZB_ZCL_CMD_ON_OFF_ON_ID : ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID;

  esp_zb_lock_acquire(portMAX_DELAY);
  esp_zb_zcl_set_attribute_val(14 + r, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                               ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, &on, false);
  esp_zb_zcl_on_off_cmd_req(&cmd);
  esp_zb_lock_release();
}

bool displayOk = false;

/************************ Temp sensor *****************************/
static bool assigned(uint64_t addr) {
  for (uint8_t p = 0; p < (sizeof(probes)/sizeof(*probes)); p++) {
    if (probes[p].addr == addr) return true;
  }
  return false;
}

static float toF(float c) { return c * 1.8f + 32.0f; }

// A reading is shown if the probe had a good read in the last 30 s, so a single
// read error doesn't blank it.
static bool fresh(const Snapshot &s, uint8_t p) {
  return s.at[p] != 0 && millis() - s.at[p] < 30000;
}

// A status tile in the right column: filled when on, outlined when off. Leaves the
// draw color set for the icon (0 inside a filled tile, 1 otherwise).
static void tile(uint8_t y, bool on) {
  u8g2.setDrawColor(1);
  if (on) { u8g2.drawRBox(98, y + 1, 30, 14, 2); u8g2.setDrawColor(0); }
  else      u8g2.drawRFrame(98, y + 1, 30, 14, 2);
}

// n lines through (cx, cy), r pixels each way, evenly spaced from angle a: 2n spokes.
// Used for the impeller (n = 2) and the snowflake (n = 3).
static void spokes(uint8_t cx, uint8_t cy, uint8_t r, uint8_t n, float a) {
  for (uint8_t k = 0; k < n; k++) {
    float u = a + k * (float)M_PI / n;
    int dx = lroundf(r * cosf(u)), dy = lroundf(r * sinf(u));
    u8g2.drawLine(cx - dx, cy - dy, cx + dx, cy + dy);
  }
}

// The status screen. Left: the tub, with the water top temperature large near the
// surface and the bottom temperature on the floor. Right, top to bottom: outside,
// circulation pump (a snowflake during freeze protection), big pump, heater with
// its outlet temperature. Everything that moves advances one step per frame (the
// display task draws 4 frames a second), and only while what it shows is running.
static void drawStatus(const Snapshot &s, uint32_t frame) {
  char buf[12];
  u8g2.clearBuffer();
  u8g2.setDrawColor(1);

  // Tub, open at the top, with the water surface: a zigzag that drifts left while
  // either pump moves the water
  u8g2.drawVLine(0, 2, 62);
  u8g2.drawVLine(95, 2, 62);
  u8g2.drawHLine(0, 63, 96);
  uint32_t drift = (relays[BIG_PUMP].on || relays[CIRC_PUMP].on) ? frame : 0;
  for (uint8_t x = 2; x <= 94; x++) {
    int t = (x + 2 + drift) % 4;
    u8g2.drawPixel(x, 3 + abs(t - 2));
  }

  // Water top: whole degrees large, the tenth small, right-aligned so the
  // decimal doesn't move between 99 and 100
  u8g2.setFont(u8g2_font_logisoso38_tn);
  if (fresh(s, P_TOP)) {
    long t10 = lroundf(toF(s.temp[P_TOP]) * 10);
    snprintf(buf, sizeof(buf), "%ld", t10 / 10);
    u8g2.drawStr(72 - u8g2.getStrWidth(buf), 50, buf);
    u8g2.setFont(u8g2_font_helvB12_tn);
    snprintf(buf, sizeof(buf), ".%ld", labs(t10 % 10));
    u8g2.drawStr(74, 50, buf);
    u8g2.drawCircle(88, 14, 2);  // °
  } else {
    u8g2.drawStr(30, 50, "--");
  }

  // Water bottom, on the floor of the tub
  u8g2.setFont(u8g2_font_6x10_tf);
  if (fresh(s, P_BOTTOM)) snprintf(buf, sizeof(buf), "%.1f\xb0", toF(s.temp[P_BOTTOM]));
  else                    snprintf(buf, sizeof(buf), "--");
  u8g2.drawStr(4, 61, buf);

  // Outside: a tree and the whole degrees (no ° if it doesn't fit, e.g. -15)
  u8g2.drawTriangle(103, 1, 99, 8, 107, 8);
  u8g2.drawTriangle(103, 4, 98, 12, 108, 12);
  u8g2.drawBox(102, 12, 3, 3);
  if (fresh(s, P_OUTSIDE)) {
    snprintf(buf, sizeof(buf), "%ld", lroundf(toF(s.temp[P_OUTSIDE])));
    if (strlen(buf) < 3) strcat(buf, "\xb0");
  } else {
    snprintf(buf, sizeof(buf), "--");
  }
  u8g2.drawStr(110, 12, buf);

  // Circulation pump: a filter canister whose pleats scroll down while it runs, or
  // during freeze protection a snowflake turning 20° a frame (it repeats every 60°)
  tile(16, relays[CIRC_PUMP].on);
  if (freezeActive) {
    spokes(113, 24, 5, 3, (frame % 3) * (float)M_PI / 9);
  } else {
    u8g2.drawRFrame(109, 19, 9, 11, 1);
    uint8_t o = relays[CIRC_PUMP].on ? frame % 3 : 2;
    for (uint8_t y = 21 + o; y <= 27; y += 3) u8g2.drawHLine(111, y, 5);
  }

  // Big pump: a 4-blade impeller, turning 30° a frame while the pump runs (the
  // blades repeat every 90°, so 3 frames make a full cycle)
  tile(32, relays[BIG_PUMP].on);
  u8g2.drawCircle(113, 40, 5);
  spokes(113, 40, 5, 2, relays[BIG_PUMP].on ? (frame % 3) * (float)M_PI / 6 : 0);

  // Heater: a flame, flickering while it burns (heater on and the big pump giving
  // it flow), and the outlet temperature
  tile(48, relays[HEATER].on);
  bool burning = relays[HEATER].on && relays[BIG_PUMP].on;
  uint8_t tip = (burning && frame % 3 == 0) ? 52 : 50;  // the tip drops every 3rd frame
  u8g2.drawTriangle(104, tip, 101, 59, 107, 59);
  u8g2.drawDisc(104, 59, 3, U8G2_DRAW_LOWER_LEFT | U8G2_DRAW_LOWER_RIGHT);
  if (fresh(s, P_HEATER)) snprintf(buf, sizeof(buf), "%ld", lroundf(toF(s.temp[P_HEATER])));
  else                    snprintf(buf, sizeof(buf), "--");
  u8g2.drawStr(110, 60, buf);

  u8g2.setDrawColor(1);
  u8g2.sendBuffer();
}

// Redraws the status screen 4 times a second, so it can animate. The panel is on
// when Hubitat's screen switch is on, and always during probe setup.
static void display_task(void *arg) {
  bool panelOn = true;
  for (uint32_t frame = 0;; frame++) {
    xSemaphoreTake(oledMutex, portMAX_DELAY);
    bool setup = setupActive;  // read under the mutex, so it can't draw over a prompt
    bool want = relays[SCREEN].on || setup;
    if (want != panelOn) {
      u8g2.setPowerSave(want ? 0 : 1);
      panelOn = want;
    }
    if (want && !setup) {
      Snapshot s = {};
      xQueuePeek(snapQ, &s, 0);
      drawStatus(s, frame);
    }
    xSemaphoreGive(oledMutex);
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

// Safety checks. Runs in its own task, started before the Zigbee wait in setup(),
// so it keeps running when the hub is unreachable. It is the only task that
// switches relays: Hubitat's commands reach it through cmdQ, and every change
// goes through setRelay(), which also reports it to Hubitat.
static void safety_task(void *arg) {
  // Subscribe to the task watchdog. The core's sdkconfig sets a 5 s timeout and a
  // panic on expiry, so if this task stops feeding it, the board resets and the
  // relays fail off.
  if (esp_task_wdt_add(NULL) != ESP_OK) {
    log_e("safety: could not subscribe to the task watchdog");
  }

  enum { PURGE_IDLE, PURGE_GAP, PURGE_RUN } purge = PURGE_IDLE;
  uint32_t purgeAt = 0;             // when the current purge step started
  uint32_t pumpLastOn = millis();   // last time the big pump was seen running
  uint32_t lastDump = 0;

  bool freeze = false;              // freeze protection active
  bool hot = false;                 // over-temperature active
  bool circWanted = false;          // what Hubitat last asked for the circ pump (off at boot)

  uint32_t toggles[5] = {};         // times of the last 5 "on" commands for the screen
  uint8_t toggleNext = 0, toggleCount = 0;

  for (;;) {
    esp_task_wdt_reset();

    // Each time the board (re)joins, tell Hubitat the real state of every relay. At
    // boot they're all off, but Hubitat still shows whatever it knew before.
    bool connected = Zigbee.connected();
    if (connected && !wasConnected) {
      for (uint8_t r = 0; r < (sizeof(relays)/sizeof(*relays)); r++) {
        setRelay(r, relays[r].on);
      }
    }
    wasConnected = connected;


    // Commands from Hubitat. Waiting here (at most 1 s) instead of sleeping means
    // a command is applied as soon as it arrives.
    RelayCmd c;
    if (xQueueReceive(cmdQ, &c, pdMS_TO_TICKS(1000)) == pdTRUE) {
      if (purge != PURGE_IDLE && c.relay != SCREEN) {
        log_i("purge: ended by a command");
        if (purge == PURGE_RUN) {
          setRelay(BIG_PUMP, false);
          delay(1000);  // let the pump stop before running the command
        }
        purge = PURGE_IDLE;
      }
      // 5 "on" commands for the screen from Hubitat within 60 s: run probe setup
      if (c.relay == SCREEN && c.on) {
        uint32_t t = millis();
        toggles[toggleNext] = t;
        toggleNext = (toggleNext + 1) % 5;
        if (toggleCount < 5) toggleCount++;
        if (toggleCount == 5 && t - toggles[toggleNext] <= 60000) {
          log_i("Screen on 5 times in 60 s: probe setup requested");
          setupRequested = true;
          toggleCount = 0;
        }
      }
      if (c.relay == CIRC_PUMP) circWanted = c.on;
      if (freeze && c.relay == CIRC_PUMP && !c.on) {
        log_w("freeze protection: circ pump stays on");
        setRelay(CIRC_PUMP, true);   // put the attribute back and tell Hubitat
      } else if (hot && c.relay == BIG_PUMP && c.on) {
        log_w("over-temperature: big pump stays off");
        setRelay(BIG_PUMP, false);   // put the attribute back and tell Hubitat
      } else {
        setRelay(c.relay, c.on);
      }
    }

    uint32_t now = millis();
    if (relays[BIG_PUMP].on) pumpLastOn = now;

    // Purge steps
    switch (purge) {
      case PURGE_IDLE:
        if (now - pumpLastOn >= PURGE_IDLE_MS) {
          log_i("purge: heater off");
          setRelay(HEATER, false);
          purge = PURGE_GAP;
          purgeAt = now;
        }
        break;
      case PURGE_GAP:
        if (now - purgeAt >= PURGE_GAP_MS) {
          log_i("purge: big pump on");
          setRelay(BIG_PUMP, true);
          purge = PURGE_RUN;
          purgeAt = now;
        }
        break;
      case PURGE_RUN:
        if (now - purgeAt >= PURGE_RUN_MS) {
          log_i("purge: big pump off");
          setRelay(BIG_PUMP, false);
          purge = PURGE_IDLE;
        }
        break;
    }

    // Freeze protection, from the outside probe. Before the first snapshot, fs is all
    // zero: the probe reads 0 °C, which turns freeze protection on until the first
    // reading arrives, and its last good read is at 0, so it goes stale after 5 min
    // of uptime.
    Snapshot fs = {};
    xQueuePeek(snapQ, &fs, 0);
    bool stale = now - fs.at[P_OUTSIDE] >= FREEZE_STALE_MS;
    float outside = fs.temp[P_OUTSIDE];  // last good reading
    bool wasFreeze = freeze;
    if (stale || outside < FREEZE_ON_C) freeze = true;
    else if (outside > FREEZE_OFF_C)    freeze = false;
    if (freeze != wasFreeze) {
      if (stale) log_w("freeze protection on (no outside reading for 5 min)");
      else       log_i("freeze protection %s (outside temperature)", freeze ? "on" : "off");
      if (!freeze) setRelay(CIRC_PUMP, circWanted);  // back to what Hubitat last asked for
    }
    if (freeze && !relays[CIRC_PUMP].on) setRelay(CIRC_PUMP, true);

    // Over-temperature, from the water top probe
    bool topStale = now - fs.at[P_TOP] >= HOT_STALE_MS;
    float top = fs.temp[P_TOP];  // last good reading
    bool wasHot = hot;
    if (topStale || top >= HOT_ON_C) hot = true;
    else if (top < HOT_OFF_C)        hot = false;
    if (hot != wasHot) {
      if (topStale) log_w("over-temperature on (no water top reading for 5 min)");
      else          log_i("over-temperature %s (water top temperature)", hot ? "on" : "off");
      if (hot && purge != PURGE_IDLE) {
        log_i("purge: ended by over-temperature");
        purge = PURGE_IDLE;
      }
    }
    if (hot && relays[BIG_PUMP].on) setRelay(BIG_PUMP, false);
    freezeActive = freeze;  // for the display
    hotActive = hot;

    if (now - lastDump >= 5000) {
      lastDump = now;
      Snapshot s;
      if (xQueuePeek(snapQ, &s, 0) == pdTRUE) {
        for (uint8_t p = 0; p < (sizeof(probes)/sizeof(*probes)); p++) {
          log_v("snap %s: %s %.2f°C, %lu ms old", probes[p].label,
                s.ok[p] ? "ok " : "bad", s.temp[p], (unsigned long)(now - s.at[p]));
        }
      } else {
        log_v("snap: no data yet");
      }
    }
  }
}

// The role -> probe address mapping, kept in flash (NVS) across reboots and uploads.
static void loadProbes() {
  Preferences prefs;
  prefs.begin("probes", true);
  for (uint8_t p = 0; p < (sizeof(probes)/sizeof(*probes)); p++) {
    char key[4];
    snprintf(key, sizeof(key), "p%u", p);
    probes[p].addr = prefs.getULong64(key, 0);
    log_i("Stored %s -> 0x%llx", probes[p].label, probes[p].addr);
  }
  prefs.end();
}

static void saveProbes() {
  Preferences prefs;
  prefs.begin("probes", false);
  for (uint8_t p = 0; p < (sizeof(probes)/sizeof(*probes)); p++) {
    char key[4];
    snprintf(key, sizeof(key), "p%u", p);
    prefs.putULong64(key, probes[p].addr);
  }
  prefs.end();
}

// True if a probe on the bus isn't in the mapping. A missing probe doesn't count:
// it reads as bad, and the safety checks handle that.
static bool newProbeOnBus(OneWire32 &ds) {
  uint64_t found[8];
  uint8_t n = ds.search(found, sizeof(found)/sizeof(*found));
  for (uint8_t i = 0; i < n; i++) {
    bool known = false;
    for (uint8_t p = 0; p < (sizeof(probes)/sizeof(*probes)); p++) {
      if (probes[p].addr == found[i]) known = true;
    }
    if (!known) {
      log_i("New probe 0x%llx on the bus: starting probe setup", found[i]);
      return true;
    }
  }
  return false;
}

// Read the mapped probes, update and publish the snapshot, report to Zigbee.
// The caller starts the conversion (ds.request() and the 750 ms wait).
static void readProbes(OneWire32 &ds, Snapshot &s) {
  for (uint8_t p = 0; p < (sizeof(probes)/sizeof(*probes)); p++) {
    float t;
    s.ok[p] = probes[p].addr != 0 && ds.getTemp(probes[p].addr, t) == 0;
    if (s.ok[p]) {
      s.temp[p] = t;
      s.at[p] = millis();
      log_v("Probe %s: %.2f°C", probes[p].label, t);
      zbTemps[p]->setTemperature(t);
    } else {
      log_v("Probe %s: read error", probes[p].label);
    }
  }
  xQueueOverwrite(snapQ, &s);  // publish this cycle's snapshot to all readers
}

static void temp_sensor_value_update(void *arg) {
  loadProbes();

  OneWire32 ds(D2);
  uint64_t addr[(sizeof(probes)/sizeof(*probes))];
  uint8_t found = ds.search(addr, (sizeof(probes)/sizeof(*probes)));
  log_i("Found %u DS18B20 sensor(s)", found);
  for (uint8_t i = 0; i < found; i++) {
    log_i("  %u: 0x%llx", i, addr[i]);
  }

  // No stored mapping (first boot, or after clearing it): ask which probe is which.
  bool empty = true;
  for (uint8_t p = 0; p < (sizeof(probes)/sizeof(*probes)); p++) {
    if (probes[p].addr != 0) empty = false;
  }
  if (empty) setupRequested = true;

  Snapshot s = {};
  uint32_t lastSearch = millis();

  for (;;) {
    // Probe setup: requested (boot, 5 "on" commands for the screen), or a new probe
    // on the bus, checked once a minute.
    bool needSetup = setupRequested.exchange(false);
    if (!needSetup && millis() - lastSearch >= 60000) {
      lastSearch = millis();
      needSetup = newProbeOnBus(ds);
    }
    if (needSetup) {
      setupActive = true;  // the display task turns the panel on and stays out of the way
      setupProbes(ds, s);
      setupActive = false;
      for (uint8_t p = 0; p < (sizeof(probes)/sizeof(*probes)); p++) {
        log_i("Probe %s -> endpoint %u -> 0x%llx", probes[p].label, 10 + p, probes[p].addr);
      }
      lastSearch = millis();
    }

    ds.request();                    // start conversion on all sensors
    delay(750);

    readProbes(ds, s);

    delay(1000);
  }
}

static void showPrompt(const char *label) {
  log_i("Probe setup: warm, cool or unplug the %s probe", label);
  if (!displayOk) return;
  xSemaphoreTake(oledMutex, portMAX_DELAY);
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "Probe setup");
  u8g2.drawStr(0, 30, "Warm, cool or unplug:");
  u8g2.setFont(u8g2_font_helvB12_tf);
  u8g2.drawStr(0, 50, label);
  u8g2.sendBuffer();
  xSemaphoreGive(oledMutex);
}

// The setup screen: the prompt, then one line per probe found so far (up to 4):
// the last four hex digits of its serial number, its temperature, and its role once
// assigned.
static void showSetup(const char *label, const Candidate *c, const float *t, const bool *ok, uint8_t n) {
  if (!displayOk) return;
  char buf[24];
  xSemaphoreTake(oledMutex, portMAX_DELAY);
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 8, "Warm, cool or unplug:");
  u8g2.setFont(u8g2_font_helvB10_tf);
  u8g2.drawStr(0, 21, label);
  u8g2.setFont(u8g2_font_6x10_tf);
  for (uint8_t i = 0; i < n && i < 4; i++) {
    unsigned id = (c[i].addr >> 8) & 0xFFFF;
    const char *role = c[i].role ? c[i].role : "";
    if (ok[i]) snprintf(buf, sizeof(buf), "%04X %5.1f\xb0 %s", id, toF(t[i]), role);
    else       snprintf(buf, sizeof(buf), "%04X   --   %s", id, role);
    u8g2.drawStr(0, 32 + i * 10, buf);
  }
  u8g2.sendBuffer();
  xSemaphoreGive(oledMutex);
}

// Search the bus once and add any new probes to c[]; returns the new count.
static uint8_t searchMore(OneWire32 &ds, Candidate *c, uint8_t n, uint8_t max) {
  uint64_t batch[8];
  uint8_t m = ds.search(batch, sizeof(batch)/sizeof(*batch));
  for (uint8_t j = 0; j < m; j++) {
    bool seen = false;
    for (uint8_t i = 0; i < n; i++) if (c[i].addr == batch[j]) seen = true;
    if (!seen && n < max) {
      c[n] = {batch[j], false, false, false, 0.0f, millis()};
      log_d("Probe setup: found 0x%llx", batch[j]);
      n++;
    }
  }
  return n;
}

static void setupProbes(OneWire32 &ds, Snapshot &s) {
  Candidate c[8];
  const uint8_t max = sizeof(c)/sizeof(*c);
  uint8_t n = 0;
  uint64_t assigned[sizeof(probes)/sizeof(*probes)] = {};
  float temps[sizeof(c)/sizeof(*c)];
  bool oks[sizeof(c)/sizeof(*c)];

  for (uint8_t role = 0; role < P_HEATER; role++) {
    for (uint8_t i = 0; i < n; i++) {
      c[i].haveBase = false;
      c[i].wasGone = false;
      c[i].lastOk = millis();
    }

    showPrompt(probes[role].label);
    bool done = false;
    while (!done) {
      n = searchMore(ds, c, n, max);

      ds.request();
      delay(750);
      readProbes(ds, s);

      log_d("Probe setup (%s):", probes[role].label);
      for (uint8_t i = 0; i < n; i++) {
        float t;
        bool ok = ds.getTemp(c[i].addr, t) == 0;
        oks[i] = ok;
        temps[i] = t;
        if (ok) {
          log_d("  0x%016llx  %6.2f°C  %6.1f°F%s", c[i].addr, t, toF(t),
                c[i].taken ? "  (assigned)" : "");
        } else {
          log_d("  0x%016llx  --%s", c[i].addr, c[i].taken ? "  (assigned)" : "");
        }

        if (c[i].taken || done) continue;
        if (ok) {
          if (c[i].wasGone || (c[i].haveBase && fabsf(t - c[i].base) > SETUP_DELTA_C)) {
            assigned[role] = c[i].addr;
            c[i].taken = true;
            done = true;
            log_i("Probe setup: %s -> 0x%llx", probes[role].label, c[i].addr);
          } else if (!c[i].haveBase) {
            c[i].base = t;
            c[i].haveBase = true;
          }
          c[i].lastOk = millis();
        } else if (millis() - c[i].lastOk >= SETUP_UNPLUG_MS) {
          c[i].wasGone = true;
        }
      }

      showSetup(probes[role].label, c, temps, oks, n);
    }
  }

  // A few more searches, so a heater probe the earlier ones missed still counts
  for (uint8_t k = 0; k < 3; k++) n = searchMore(ds, c, n, max);
  for (uint8_t i = 0; i < n; i++) {
    if (!c[i].taken) {
      assigned[P_HEATER] = c[i].addr;
      log_i("Probe setup: %s -> 0x%llx", probes[P_HEATER].label, c[i].addr);
      break;
    }
  }

  for (uint8_t p = 0; p < (sizeof(probes)/sizeof(*probes)); p++) {
    probes[p].addr = assigned[p];
  }
  saveProbes();
}

/********************* Arduino functions **************************/
void setup() {
  pinMode(3, OUTPUT);  digitalWrite(3, LOW);   // enable RF switch (active low)
  pinMode(14, OUTPUT); digitalWrite(14, LOW);  // LOW = onboard ceramic antenna

  // Messages go through the core's log macros (log_e, log_w, log_i, log_d, log_v),
  // which print only up to the level set at build time (DebugLevel in the FQBN):
  //   error    failures the board can't recover from by itself
  //   warning  problems that were handled (a dropped command, a stale probe)
  //   info     events: relays, purge, safety checks, joins, probe setup steps
  //   debug    the probe list on every probe setup cycle
  //   verbose  every probe reading, and the snapshot every 5 s
  // setDebugOutput() sends them to the USB serial port.
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  Wire.begin();  // D4 = SDA, D5 = SCL on the XIAO
  displayOk = u8g2.begin();
  if (!displayOk) {
    log_e("SH1106 not found - check wiring/address");
  }

  // Init button switch
  pinMode(button, INPUT_PULLUP);

  // Relays start off
  for (uint8_t r = 0; r < (sizeof(relays)/sizeof(*relays)); r++) {
    if (relays[r].pin == NO_PIN) continue;
    pinMode(relays[r].pin, OUTPUT);
    digitalWrite(relays[r].pin, LOW);
  }
  zbRelay1.onPowerOutletChange([](bool on) { onRelayChange(0, on); });
  zbRelay2.onPowerOutletChange([](bool on) { onRelayChange(1, on); });
  zbRelay3.onPowerOutletChange([](bool on) { onRelayChange(2, on); });
  zbRelay4.onPowerOutletChange([](bool on) { onRelayChange(3, on); });

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

  cmdQ = xQueueCreate(8, sizeof(RelayCmd));
  if (!cmdQ) { log_e("command queue alloc failed"); ESP.restart(); }

  log_i("Starting Zigbee...");
  // When all EPs are registered, start Zigbee in Router mode
  if (!Zigbee.begin(ZIGBEE_ROUTER)) {
    log_e("Zigbee failed to start! Rebooting...");
    ESP.restart();
  } else {
    log_i("Zigbee started successfully!");
  }

  // One-slot mailbox for Snapshot; see the comment at struct Snapshot.
  snapQ = xQueueCreate(1, sizeof(Snapshot));
  if (!snapQ) { log_e("snapshot queue alloc failed"); ESP.restart(); }

  oledMutex = xSemaphoreCreateMutex();
  if (displayOk) xTaskCreate(display_task, "display", 4096, NULL, 5, NULL);

  // Start Temperature sensor reading task
  xTaskCreate(temp_sensor_value_update, "temp_sensor_update", 4096, NULL, 10, NULL);

  xTaskCreate(safety_task, "safety", 4096, NULL, 11, &safetyTask);

  log_i("Connecting to network");
  while (!Zigbee.connected()) {
    delay(100);
  }
  log_i("Connected to network");

  // Set reporting interval for temperature measurement in seconds, must be called after Zigbee.begin()
  // min_interval and max_interval in seconds, delta in °C
  // if min = 1 and max = 0, reporting is sent only when temperature changes by delta
  // if min = 0 and max = 10, reporting is sent every 10 seconds or temperature changes by delta
  // if min = 0, max = 10 and delta = 0, reporting is sent every 10 seconds regardless of temperature change
  for (uint8_t p = 0; p < (sizeof(probes)/sizeof(*probes)); p++) {
    zbTemps[p]->setReporting(1, 0, 1);
  }
}


// Called by the library when Hubitat sends a command for an output, in the Zigbee
// task: the command is queued for the safety task. A change made from the safety
// task itself (through setState(), which nothing calls today) is applied directly.
static void onRelayChange(uint8_t r, bool on) {
  if (xTaskGetCurrentTaskHandle() == safetyTask) {
    setRelay(r, on);
  } else {
    RelayCmd c = {r, on};
    if (xQueueSend(cmdQ, &c, 0) != pdTRUE) {
      log_w("%s: command queue full, command dropped", relays[r].name);
    }
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
        log_i("Resetting Zigbee to factory and rebooting in 1s.");
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
