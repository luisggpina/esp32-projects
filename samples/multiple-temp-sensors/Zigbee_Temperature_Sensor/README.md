# Multiple DS18B20 sensors + OLED, reporting over Zigbee to Hubitat

A Seeed Studio XIAO ESP32-C6 running as a Zigbee router. It reads up to four DS18B20
temperature sensors on one 1-Wire bus, shows each reading and their average on an SH1106
OLED in °F, and sends the average to Hubitat as a Zigbee temperature sensor.

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

All the sensors share one bus. Each DS18B20 has a unique 64-bit address, and the sketch
prints the addresses at boot:

```
Found 4 DS18B20 sensor(s)
  0: 0x...
  1: 0x...
```

The sensor numbering (1-4 on the screen) follows the order `search()` finds them in,
which depends on their addresses, not on how they're wired.

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
| Erase All Flash Before Sketch Upload | Disabled once paired |
| USB CDC On Boot | Enabled |

## What the sketch does

- Every ~1.75 s, it starts a conversion on all sensors, waits 750 ms, and reads each one
  by its address.
- The **average of the sensors that read successfully** goes to Zigbee endpoint 10
  (Temperature Measurement). If every read fails, nothing is sent that cycle.
- **Screen:** one small line per sensor (`1: 72.5 F`, or `1: --` on a read error), with
  the average in large text at the bottom (`Avg 72.4F`).
- Reported range is −55 to 125 °C, with a tolerance of 0.5 °C. Reporting is
  `setReporting(1, 0, 1)`: at most once per second, and only when the value changes by
  1 °C or more.

## BOOT button

- **Short press:** reports the temperature immediately.
- **Hold for more than 3 s:** Zigbee factory reset. This removes the board from the
  network, and you'll need to pair it again.

## Pairing with Hubitat

1. For the first upload, set **Erase All Flash Before Sketch Upload** to Enabled. Set it
   back to Disabled before any later upload, or the upload will wipe the pairing.
2. In Hubitat's Zigbee settings, turn on **Strict Zigbee 3.0**. It only affects joining.
3. **Double Luck Voodoo:** go to **Settings → Zigbee Details → Rebuild Network**, wait
   until the button is clickable again, then run **Rebuild Network** a second time.
4. Go to **Devices → Add Device → Zigbee** and wait until Hubitat says it's ready.
5. Plug the board in, **physically touching the hub**.
6. The serial output (115200 baud) shows:

   ```
   Joined network successfully (Extended PAN ID: ..., PAN ID: 0x...., Channel:.., Short Address: 0x....)
   ```

7. On the new device's page, set the driver to
   **Generic Zigbee Temperature/Humidity Sensor**.
8. Click **Configure**. From then on, the temperature updates on its own.

Hubitat shows a single temperature, which is the average of the sensors.
