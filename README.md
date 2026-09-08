# Desert Race Logging & Vehicle Communication Device

ESP32 firmware for a desert-race vehicle logger: GNSS-based corrected race
distance against a raw-NMEA reference track, point-geofence checkpoint
detection, SD logging, SIM800L SMS, LoRa checkpoint/Give-Way messaging, an
OLED status display, and a Wi-Fi settings/file-manager interface.

See [`ARCHITECTURE.md`](ARCHITECTURE.md) for the full design, the OLED
DATA-page layout, and the ReferenceMap dedup pipeline.

## Current increment

This is Phase 1 (hardware foundation) plus the ReferenceMap indexer and
GeoFencing.txt parser groundwork for Phase 2/3, per the spec's own phased
build-out plan. See `ARCHITECTURE.md` §4 for exactly what's in and what's
deferred to later phases.

## Build

This is a [PlatformIO](https://platformio.org/) project targeting the
`esp32dev` board (ESP32 WROOM).

```
pio run            # build
pio run -t upload  # build + flash
pio device monitor  # serial monitor at 115200
```

> Note: this environment's outbound network policy blocks PlatformIO's
> package registry, so the toolchain/library packages could not be
> downloaded and a full `pio run` could not be verified here. The code was
> reviewed by hand against the actual Adafruit_SH110X / TinyGPSPlus /
> ESP32-Arduino-core APIs it calls; please run `pio run` on a machine with
> normal internet access as the first verification step, and report back
> any compiler errors so they can be fixed immediately.

## SD card layout expected at boot

```
/GeoFencing.txt      point geofences: latitude,longitude,distance_m,label
/ReferenceMap.log     raw NMEA recon lap (optional until you have one; the
                      device will boot and log live GNSS without it, but
                      corrected-distance route matching needs it)
/route/               auto-built index/cache (do not hand-edit)
```

SD initialization is a hard startup dependency: if it fails, the OLED
INIT page shows a retry counter and normal race operation does not begin
until SD comes up.

## Laptop companion tool

[`companion/index.html`](companion/index.html) is a separate, standalone
local web page (open it directly in a browser — no install, no server, no
connection to the device) for viewing/replaying a downloaded race log and
for editing `GeoFencing.txt` / trimming-merging `ReferenceMap.log` before
copying them onto the SD card. See [`companion/README.md`](companion/README.md).
