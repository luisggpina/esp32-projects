# Multiple DS18B20 sensors on separate Zigbee endpoints, with Hubitat

A Seeed Studio XIAO ESP32-C6 running as a Zigbee router. It reads up to four DS18B20
temperature sensors on one 1-Wire bus and exposes each one as its own Zigbee Temperature
Measurement endpoint. A Hubitat parent driver turns each endpoint into its own child
device. The board also shows all four readings on an SH1106 OLED in °F.

Built on [`../../Zigbee_Temperature_Sensor`](../../Zigbee_Temperature_Sensor). See that
README for the toolchain versions.

## Wiring

| Part | XIAO pin |
|---|---|
| DS18B20 data (all sensors) | D2 (GPIO2) |
| DS18B20 power / ground | 3V3 / GND |
| 1-Wire pull-up | 4.7 kΩ from D2 to 3V3 (one for the whole bus) |
| SH1106 OLED SDA | D4 (GPIO22) |
| SH1106 OLED SCL | D5 (GPIO23) |
| SH1106 OLED power / ground | 3V3 / GND |

The OLED uses I2C address `0x3C` and the board's default I2C pins, so `Wire.begin()` takes
no arguments.

## Libraries

- **esp32-ds18b20** by junkfix (`OneWireESP32.h`)
- **Adafruit GFX Library**
- **Adafruit SH110X**
- `Zigbee.h` comes with the esp32 core

## Tools settings

| Setting | Value |
|---|---|
| Board | XIAO_ESP32C6 |
| Zigbee mode | Zigbee ZCZR (coordinator/router) |
| Partition Scheme | Zigbee ZCZR 4MB with spiffs |
| Erase All Flash Before Sketch Upload | Enabled for the first upload, then Disabled |
| USB CDC On Boot | Enabled |

## Sensors and endpoints

Each entry in the `probes[]` table maps one DS18B20 to one Zigbee endpoint:

| Table entry | Zigbee endpoint | Hubitat child |
|---|---|---|
| `probes[0]` | 10 (0x0A) | probe 1 |
| `probes[1]` | 11 (0x0B) | probe 2 |
| `probes[2]` | 12 (0x0C) | probe 3 |
| `probes[3]` | 13 (0x0D) | probe 4 |

Every DS18B20 has a unique 64-bit address. Entries left at `0x0` are filled at boot with
the sensors found on the bus, in `search()` order, which depends on the addresses rather
than on the wiring. The boot output shows the result:

```
Found 4 DS18B20 sensor(s)
  0: 0x...
Probe 1 -> endpoint 10 -> 0x...
Probe 2 -> endpoint 11 -> 0x...
```

To keep each sensor on the same endpoint when sensors are added, removed or replaced,
copy the addresses into the table:

```cpp
Probe probes[] = {
  {"1", 0x28...ULL},
  {"2", 0x28...ULL},
  {"3", 0x28...ULL},
  {"4", 0x28...ULL},
};
```

To find out which sensor is which, hold one sensor at a time and watch which reading
climbs.

Each probe needs a matching `ZigbeeTempSensor` in `zbTemps[]`. A `static_assert` stops the
build if the two arrays have different lengths. The OLED has room for four probes.

## What the sketch does

- Every ~1.75 s, it starts a conversion on all sensors, waits 750 ms, and reads each probe
  by its address.
- Each successful reading goes to that probe's endpoint. A failed read is logged, and that
  endpoint keeps its last value.
- **Screen:** one line per probe (`1 72.5F`, or `1 --` on a read error).
- Reported range is −55 to 125 °C, with a tolerance of 0.5 °C.
- Reporting: on its own, each endpoint reports at most once per second, and only when the
  value changes by 1 °C or more. Hubitat's **Configure** replaces this with the driver's
  settings (below).

## BOOT button

- **Short press:** every endpoint reports immediately.
- **Hold for more than 3 s:** Zigbee factory reset. This removes the board from the
  network, and you'll need to pair it again.

## Hubitat driver

[`../hubitat/esp32c6-multi-ds18b20.groovy`](../hubitat/esp32c6-multi-ds18b20.groovy) is a
parent driver ("ESP32-C6 Multi DS18B20"). It:

- creates one child device per endpoint, named "<parent name> probe 1" to "probe 4", using
  Hubitat's built-in **Generic Component Temperature Sensor** driver
- on **Configure**, binds the Temperature Measurement cluster (`0x0402`) on each endpoint
  to the hub, and sets up reporting: at least every 10 s, at most every 600 s, on a change
  of 0.5 °C
- routes each incoming temperature report to the child for its endpoint, converting to the
  hub's temperature scale
- on **Refresh** (parent or child), reads the temperature from the endpoints

Install it under **Drivers Code → New Driver**: paste the file and **Save**.

## Pairing with Hubitat

1. Install the driver (above).
2. If the board was paired before with a different set of endpoints, remove that device in
   Hubitat. The endpoint list is only read when the device joins.
3. Upload with **Erase All Flash Before Sketch Upload** set to Enabled. Set it back to
   Disabled before any later upload, or the upload will wipe the pairing.
4. In Hubitat's Zigbee settings, turn on **Strict Zigbee 3.0**. It only affects joining.
5. **Double Luck Voodoo:** go to **Settings → Zigbee Details → Rebuild Network**, wait
   until the button is clickable again, then run **Rebuild Network** a second time.
6. Go to **Devices → Add Device → Zigbee** and wait until Hubitat says it's ready.
7. Plug the board in, **physically touching the hub**.
8. The serial output (115200 baud) shows:

   ```
   Joined network successfully (Extended PAN ID: ..., PAN ID: 0x...., Channel:.., Short Address: 0x....)
   ```

9. On the new device's page, set the driver to **ESP32-C6 Multi DS18B20** and click
   **Save Device**.
10. Click **Configure**. The four child devices appear, and from then on each one updates
    on its own.
