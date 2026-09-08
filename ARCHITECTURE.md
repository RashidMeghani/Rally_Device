# Desert Race Logging Device — Architecture (updated for Revision 3)

This document reflects the full spec (Revision 2 baseline + Revision 3
clarifications, which are authoritative wherever the two conflict). It is
the pre-coding design update requested before implementing the OLED DATA
page and the ReferenceMap dedup pipeline, and it stays current as later
phases (SMS, LoRa, Give Way, WebManager) are implemented.

## 1. Confirmed hardware/data decisions carried into this design

- OLED: SH1106, 128x64, I2C at **0x3C**, SCL=GPIO22, SDA=GPIO21, via
  `Adafruit_SH110X`. Four pages only: SPLASH, INIT, DATA, SETTINGS.
- LoRa RA-02: carrier fixed at **433 MHz** (`AppConst::LORA_FREQ_HZ`),
  centralized/configurable, shares SPI with SD (CS13 vs CS5).
- `GeoFencing.txt`: plain CSV, no header, row order
  `latitude,longitude,distance_from_start_m,label`, file order == race
  order. Points, never gate lines.
- `ReferenceMap.log`: raw NMEA, authoritative, append-only, never rewritten
  by any preprocessing step.
- Derived route segment/index files hold **deduplicated position records
  only** (see §3) — they are a cache, not a second source of truth.
- HTML settings page (WebManager, later phase) must expose M8N **baud
  rate**, **refresh/update rate** (≤10 Hz), and **enabled NMEA sentence
  set**. `ConfigManager`/`GpsManager` already implement the persistence
  and receiver-side apply path for this (`AppConfig::gnssBaud`,
  `gnssRateHz`, `gnssEnabledSentences`), so WebManager only needs to become
  a thin HTTP form over `ConfigManager::save()` + `GpsManager::applySettings()`
  when it is built.

## 2. OLED — four pages and the DATA page layout

### 2.1 Page state machine

```
SPLASH  --3s (non-blocking timer)-->  INIT
INIT    --all one-time boot steps done-->  DATA
DATA    <---> SETTINGS   (Key 2 held 5s, either direction)
```

`DisplayManager` is a pure view: `setPage()`, `addInitLine()`,
`updateDataModel()`, `setSettingsInfo()` just update in-RAM state (cheap,
no I2C traffic). Actual redraw + I2C flush happens only inside `loop()`,
throttled to `OLED_REFRESH_INTERVAL_MS` (200 ms / ~5 Hz) — decoupled from
the up-to-10 Hz GNSS rate so a ~20-30 ms I2C frame flush can never become
the bottleneck for GNSS parsing.

### 2.2 DATA page — concrete 128x64 layout

The hand sketch (Figure 1 in the brief) groups fields into a left column
(ahead distance, crossing time, speed, logging symbol, covered distance)
and a right column (ahead ID, geofence label/distance, satellites,
accuracy). The layout below keeps that semantic grouping but reflows it
into fixed pixel zones sized for the default Adafruit-GFX font (6x8 px at
size 1, 12x16 px at size 2), so no field can overwrite a neighbor:

| Zone (y range)     | Left content                          | Right content                    |
|--------------------|----------------------------------------|-----------------------------------|
| Row A: y=0–8 (sz1)  | `A:<ft>` — Field 1, ahead distance, x=0, ≤12 chars | `ID:<id>` — Field 6, ahead device ID, x=74, ≤9 chars |
| Row B: y=9–17 (sz1) | `T:HH:MM:SS.cc` — Field 2, crossing time, x=0, full width (≤21 chars) | — |
| *divider line*     | y=17, full width                       |                                   |
| Row C: y=18–34      | Field 3 speed: number size2 x=0 (≤6 chars) + `km/h` size1 at x=40,y=26 | `GF:<label>` — Field 7, x=74,y=18, ≤9 chars<br>`D:<m>m` — Field 8, x=74,y=26, ≤9 chars |
| *divider line*     | y=35, full width                       |                                   |
| Row D: y=36–44 (sz1)| `L` — Field 4, x=0, shown only while logging (blank otherwise) | `Dist:<m>m` — Field 5, corrected distance, x=10, ≤18 chars |
| *divider line*     | y=45, full width                       |                                   |
| Row E: y=46–54 (sz1)| `Sats:<n>` — Field 9, x=0, ≤12 chars    | `Acc:<m>m` — Field 10, x=74, ≤9 chars |

All ten mandatory fields (spec §29) are present; none omitted. Alignment
was optimized for a 128x64 grid rather than mirroring the sketch pixel-for-
pixel, per the spec's explicit allowance ("the attached hand sketch is the
layout reference, not a pixel-perfect requirement").

Placeholder rule: every field has a `*Valid` flag in `RaceDataModel`. When
false, the field renders a fixed placeholder (`--`) instead of a stale
number — except Field 4 (`L`), which is a boolean indicator, not a data
value, and is correctly blank (not `--`) when not logging. Every field is
also drawn through `drawClipped()`, which truncates to that zone's maximum
character count so a long device ID or label can never bleed into an
adjacent field.

Implemented in `src/DisplayManager.{h,cpp}` (`drawData()`).

## 3. ReferenceMap raw-NMEA preservation + dedup design

**Problem.** `ReferenceMap.log` must keep every enabled NMEA sentence
verbatim (raw authoritative source). But a single physical GNSS fix is
reported through *multiple* sentence types in the same epoch — GNRMC and
GNGGA both carry a full lat/lon for the same instant. If every position-
bearing sentence were inserted into the route-search index, the same
physical fix would be duplicated once per sentence type, corrupting
cumulative distance (double counting) and bloating the index.

**Design.** `ReferenceMapIndexer` (`src/route/ReferenceMapIndexer.{h,cpp}`)
opens `ReferenceMap.log` **read-only** and never modifies it — every
sentence in the raw file, including ones irrelevant to indexing (VTG, GSA,
GSV, or the disputed "GNGTV" identifier), is preserved forever as the
authoritative replay source. Deduplication happens only in a separate
pass that streams the file once:

1. GNRMC and GNGGA both carry the same UTC "hhmmss.ss" time-of-day field
   for a given epoch. Sentences sharing that exact time-field string are
   treated as **the same physical fix**, regardless of which sentence
   type(s) reported it.
2. While consecutive sentences share the same time field, only the first
   one that yields a *valid* lat/lon (RMC status `A`, or GGA fix quality
   `>0`) is buffered as the pending point for that epoch. Any later
   sentence in the same group is not re-parsed into a second point
   (first-valid-wins).
3. When the time field changes — a new epoch begins — the previously
   buffered point (if any) is emitted exactly once: cumulative distance is
   advanced via haversine from the last emitted point, and the row is
   appended to the current 2 km segment file.
4. End of file flushes whatever epoch is still pending.

This makes the dedup key "same UTC time-of-day field" rather than "same
sentence type," so it is agnostic to which sentence types are enabled —
it keeps working unchanged if GLL or GNS sentences were ever added, and it
sidesteps the GNRMC/"GNGTV" identifier ambiguity entirely: VTG (or
whatever "GNGTV" turns out to literally be) carries no lat/lon in
NMEA-0183, so it can never contribute a position and is simply skipped for
indexing purposes — the ambiguity only matters for live-speed decoding,
not for ReferenceMap indexing.

**On-disk layout (SD):**

```
/ReferenceMap.log        raw NMEA, authoritative, untouched
/route/index.hdr         {sourceSize, fingerprintCrc, pointCount,
                          segmentCount, totalDistanceM, builderVersion}
/route/index.csv         one row per ~2 km segment:
                          segIndex,startDistanceM,endDistanceM,fileName
/route/seg_00000.csv ...  one row per deduplicated point:
                          lat,lon,cumulativeDistanceM
```

`index.csv` is small (~125 rows for a 250 km track) and is meant to be
kept fully resident in RAM by the future `RouteMatcher`; individual segment
files are opened on demand (1-2 at a time), so neither the indexer nor the
runtime matcher ever loads the full ~250 km raw file into RAM or rescans it
per GNSS fix.

**Rebuild trigger.** `needsRebuild()` compares a cheap fingerprint (file
size + CRC32 of just the first/last 256 bytes) against the stored header,
rather than re-hashing the full file on every boot — hashing the whole
~250 km file at every startup would itself reintroduce the "don't scan the
whole route" cost this index exists to avoid. This is a documented
trade-off, not a silent one: a same-size, interior-only edit to
`ReferenceMap.log` would not be detected. In practice the file is produced
by a single recon pass and is either replaced wholesale or only appended
to, both of which this fingerprint reliably catches; a full-file CRC mode
can be substituted later if stricter detection is ever required.

Build/commit is atomic per file: segment/index/header files are written to
`*.tmp` and only `rename()`d into place after a successful write, so a
power loss mid-build cannot leave a half-written index that looks valid on
the next boot.

## 4. Firmware layout (this increment)

```
include/
  PinConfig.h        confirmed pin map
  AppConstants.h      thresholds/paths from the spec baseline
src/
  ConfigManager.*      NVS-backed config incl. M8N HTML settings
  GpsManager.*         Serial2 GNSS, raw pass-through, M8N settings apply
  GeoFenceManager.*     GeoFencing.txt parser + skip-passed/reset logic
  DisplayManager.*      SH1106 4-page non-blocking view (this increment's
                        main deliverable)
  route/ReferenceMapIndexer.*   dedup + segment/index builder (this
                        increment's other main deliverable)
  util/NmeaUtil.*       shared NMEA parsing helpers
  main.cpp              boot state machine + wiring
```

Per the spec's own phased plan (§24), this increment covers Phase 1
(hardware foundation: pins, buttons, OLED all four pages, SD/SPI
arbitration, GNSS at configured baud) plus the ReferenceMap indexer and
GeoFencing.txt parser groundwork that Phase 2/3 build on. SIM800L/SMS,
LoRa transport, Give Way/Overtake, WebManager/HTML server, NeoPixel matrix,
and the race-logging/geofence-crossing state machine are intentionally
deferred to their listed phases — generating them now, ahead of hardware
bring-up feedback on this increment, would risk exactly the "one monolithic
sketch" the spec's master prompt says not to produce.

`AppController` is, for now, the small boot state machine in `main.cpp`
(SPLASH → INIT → [SD_ERROR retry loop] → READY) rather than a separate
class file — it will be promoted to its own module once the race-runtime
state machine (start/logging/recovery, spec §10) is added in the next
phase, since factoring it out before that state machine exists would be
premature.

## 5. Remaining genuinely open items

See the chat response for the current short list — kept there rather than
duplicated here so there is a single place it's tracked as it gets
resolved.
