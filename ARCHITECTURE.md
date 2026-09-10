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
| Row C: y=18–34      | Field 3 speed: number size2 x=0 (≤6 chars) + `km/h` size1 at x=40,y=26 | `GF:<label>` — Field 7, x=74,y=18, ≤9 chars<br>`D:<m>m` — Field 8, x=74,y=26, ≤9 chars |
| Row D: y=36–44 (sz1)| `Dist:<m>m` — Field 5, corrected distance, x=0, left-aligned, ≤21 chars | — |
| Row E: y=46–54 (sz1)| `Sats:<n>` — Field 9, x=0, ≤12 chars    | `Acc:<m>m` — Field 10, x=74, ≤9 chars |
| Row F: y=55–63 (sz1)| battery voltage (bonus field), x=0      | `L` — Field 4, right-aligned at x=122, directly below accuracy; shown only while actively writing |

No horizontal divider lines between rows (owner revision - removed for a
cleaner look; vertical spacing alone keeps the rows visually separated).

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

## 4. Firmware layout (current)

```
include/
  PinConfig.h        confirmed pin map
  AppConstants.h      thresholds/paths from the spec baseline
src/
  ConfigManager.*      NVS-backed config incl. M8N HTML settings
  GpsManager.*         Serial2 GNSS, raw pass-through, M8N settings apply
  GeoFenceManager.*     GeoFencing.txt parser + skip-passed/reset logic
  DisplayManager.*      SH1106 4-page non-blocking view
  LogManager.*          race-log lifecycle (open/write/close per section 10)
  ButtonManager.*       Key1/2/4 debounce + staged long-press (section 13)
  BatteryManager.*      2S Li-ion ADC sensing (section 17)
  AppController.*       race/recovery state machine + geofence-crossing
                        orchestration (sections 8, 10, 11) - see its header
                        comment for the honest scope note on what's
                        approximated pending RouteMatcher
  route/ReferenceMapIndexer.*   dedup + segment/index builder
  util/NmeaUtil.*       shared NMEA parsing helpers
  main.cpp              boot sequence (SPLASH -> INIT -> [SD_ERROR retry] ->
                        READY) + wiring; delegates to AppController::loop()
                        once READY
```

Per the spec's own phased plan (§24): Phase 1 (hardware foundation),
Phase 2 (ReferenceMap indexer/GeoFencing parser), and Phase 3 (point
geofence manager, race-log lifecycle, the race/recovery state machine) are
now implemented. SIM800L/SMS, LoRa transport, Give Way/Overtake,
WebManager/HTML server, and the NeoPixel matrix (Phases 4-7) remain
deliberately deferred — generating them now, ahead of hardware bring-up
feedback on what exists, would risk exactly the "one monolithic sketch"
the spec's master prompt says not to produce.

`AppController` is now its own class (promoted out of `main.cpp` per the
plan in the previous revision of this document), owning the race stage
machine (`WAIT_START -> ACTIVE -> STOPPED -> FINISHED`) and closest-
approach geofence-crossing detection. It does **not** yet implement true
reset-recovery (skip-passed-on-reset needs RouteMatcher, not built yet) —
see `AppController.h`'s header comment for exactly what's approximated and
why, rather than silently claiming section 8/10's recovery rule is done.

## 5. Whole-system architecture (all managers, not just Phase 1)

The rest of this document covers every module in spec §19, not just what's
been coded so far — this is the "architecture approval" deliverable the
master prompt asks for before more modules are generated incrementally.

### 5.1 Confirmed since the last revision of this document

- 8x8 matrix is a genuine NeoPixel (single-wire, GPIO15,
  `Adafruit_NeoPixel`, `NEO_GRB+NEO_KHZ800`) — owner-confirmed, not DotStar.
- A second, laptop-side tool is in scope **in addition to** the ESP32-hosted
  settings page: a standalone local web page (no server, no install) doing
  (a) race-log viewer/replay and (b) GeoFencing.txt/ReferenceMap.log
  editing. This is new scope beyond the original brief, kept in its own
  `companion/` directory since it never runs on the ESP32 and has no
  network dependency on the device at all (see §11.2).

### 5.2 Shared-resource ownership (all managers)

- **SPI** (MOSI23/MISO19/SCK18, shared LoRa+SD): both `SD` and
  `arduino-LoRa` manage CS per-transaction internally; since everything
  runs single-threaded inside `loop()`, arbitration reduces to never
  interleaving SD and LoRa calls across execution contexts — naturally
  satisfied by run-to-completion function calls, documented rather than
  left implicit (spec §3 engineering check).
- **I2C** (SCL22/SDA21): `DisplayManager` only.
- **UART2 / UART**: `GpsManager` / `GsmManager`, exclusively.
- **Single-wire NeoPixel** (GPIO15): `DisplayManager` owns the matrix too
  (renamed conceptually to "OLED+matrix view layer"), since both the OLED
  and the matrix are pure output devices driven by the same `AppState`
  (Give Way session state in particular) and both need the same
  non-blocking "cheap struct copy in, throttled hardware flush in
  `loop()`" discipline. `OvertakeManager` owns the Give Way *logic* (state
  machine, session, peer selection) and only ever writes into `AppState`;
  it never touches NeoPixel registers directly.
- **NVS**: `ConfigManager` only writes; everyone else reads via `get()`.
- **SD subtrees**: `/GeoFencing.txt`, `/ReferenceMap.log` (read-only to
  firmware), `/route/*` (RouteIndex), `/races/*` (LogManager),
  `/logs/lora.log` + `/logs/sms_pending.log` (LoRaTransport/GsmManager).
  `WebManager`'s file manager is the only module allowed arbitrary paths,
  and its write endpoints are rejected (409) while a race log is actively
  open — recommendation, not stated in the spec, to stop a laptop-triggered
  delete/rename from racing a live SD write.
- **`AppState`** (corrected distance, current geofence target,
  logging-active, current fix, Give Way session): owned by `AppController`;
  single-writer-per-field (e.g. only `GeofenceManager` writes corrected
  distance/current target, only `LogManager` writes logging-active, only
  `OvertakeManager` writes Give Way session fields).

### 5.3 Startup / race / recovery state machine (full)

```
BOOT_SPLASH (3s, non-blocking)
   -> INIT (config, display, buttons, buzzer-off, GNSS, GSM, LoRa, SD, route/geofence load)
        -> SD_ERROR (non-blocking retry loop; race operation does not begin) -> back to INIT on success
        -> RACE_WAIT_START
RACE_WAIT_START
   -- START geofence crossed normally ----------------------> RACE_ACTIVE (new log file)
   -- OR valid route match acquired mid-route (reset, corrected distance > ~0) -> RACE_ACTIVE (new log file, skip-passed already applied)
RACE_ACTIVE
   -- speed <= 2 km/h continuously for 20 min --> RACE_STOPPED (log flushed+closed)
   -- final geofence crossed -------------------> RACE_FINISHED (log flushed+closed, no reopen)
RACE_STOPPED
   -- speed > 2 km/h resumes --------------------> RACE_ACTIVE (NEW log file)
RACE_FINISHED -> terminal for this run
```

Two distinct log-open triggers into `RACE_ACTIVE` (normal START crossing
vs. recovery-match-acquired) because the spec requires recovery to restart
logging "without requiring the old start point" (§8/§10) — a materially
different edge than the normal start-line crossing.

### 5.4 Point-geofence algorithm (full, incl. skip/reset)

Per-tick in `GeofenceManager::update(correctedDistance, lat, lon, speedKmh)`:
1. `target = points[nextIndex]`; none left → race-finished (AppController).
2. `dist = haversine(live, target)`, every tick.
3. `dist <= 100m` → show `target.label` (OLED Field 7).
4. `dist <= 50m` → show `dist` (OLED Field 8); track a small rolling window
   of `dist` samples to detect the local minimum (closest approach) rather
   than requiring a zero-distance reading.
5. Crossing accepted only if `speedKmh > 10` (normal checkpoints).
   **Owner revision, supersedes the original Key 4 "force start" idea**:
   Key 4 no longer bypasses this detection at all. It's now a standalone
   action independent of the geofence pipeline entirely - a quick tap
   (released before 1s) is the Give Way ahead-driver ack pulse; a 1.5s
   hold toggles a manual log start/stop directly on `LogManager`, with
   none of the marking/distance-snap/SMS/LoRa dispatch a real crossing
   does (see §5.5 and `AppController.h`'s header comment for exactly what
   a button-started log does and doesn't trigger).
6. On accepted crossing: capture GNSS time from the closest-approach
   sample, latch `passed=true`, snap corrected distance to
   `target.distanceFromStartM`, dispatch SMS+LoRa without blocking, advance
   `nextIndex`. (Implemented in `GeoFenceManager.cpp`.)
7. **Skip/reset**: whenever `AppController` acquires a fresh valid route
   match after not having one, `skipPassedBefore(correctedDistance)` marks
   every point behind it as passed without dispatching their events, moves
   `nextIndex` to the first still-unpassed point ahead, never backward.
   (Implemented.)

### 5.5 Race-log lifecycle (LogManager)

- **Open** on: normal START crossing, recovery match acquired mid-route,
  movement resuming after a 20-min auto-close (while not yet finished), or
  a manual Key 4 long-press toggle (owner revision, §5.4) - the last one
  goes straight to `LogManager` and skips the geofence pipeline entirely
  (no marking, no distance snap, no SMS/LoRa). A subsequent real START
  crossing won't reopen a log a manual toggle already has running.
- **Write**: raw NMEA buffered in a ~2 KB RAM ring buffer, flushed at
  ~75%-full or every 2s (whichever first) — bounds both worst-case
  power-loss data loss and SD write frequency; write rate follows the
  *configured* log rate, independent of GNSS output rate.
- **Close**: 20-min continuous-stop timeout (flush+close, reopen on
  resume), or final geofence (flush+close, permanently — no reopen even if
  the vehicle keeps moving post-finish; requires `LogManager` to know
  "finished," not just "stopped").
- **Filename** (flags the no-RTC gap, §1): `/races/DD-MM-YYYY HH.MM.SS.log`
  in **local time** (UTC offset applied, §5.11), e.g.
  `/races/09-09-2026 17.47.45.log`. The time separators are dots, not
  colons: `:` is a reserved character on FAT and cannot appear in an SD
  filename. (Sub-second precision is kept for the OLED crossing time,
  which the spec requires, but is not part of the filename.) Falls back to `/races/NoTime-<millis>.log` if a log must open
  before any GNSS time fix exists, rather than a retroactive rename of an
  actively-written file (FAT rename-while-open is its own failure mode) —
  recommendation pending confirmation.
- **'L' indicator**: bound to *actively writing* (file open **and** above
  the 2 km/h threshold), not merely file-open — dropping below 2 km/h
  pauses writing immediately while the file stays open until the 20-minute
  timeout, and spec §10 says the indicator goes away when logging is
  "paused/stopped".

### 5.6 Persistent pending-SMS design (GsmManager)

```cpp
struct SmsRecord {
  uint32_t id;           // monotonic
  char phone[16];
  char body[140];         // GSM 7-bit budget
  uint32_t createdAtMs;
  uint8_t attempts;        // in-RAM only; resets on reboot safely (delivered==true never resent)
  bool delivered;
};
```
Append-only JSON-lines journal at `/logs/sms_pending.log` (SD, not NVS —
NVS is wear-limited flash meant for small infrequent config, not a queue
written 5x per checkpoint event). In-RAM list mirrors the file; the file
is rewritten/compacted only when entries are delivered and removed, not on
every attempt-count bump, bounding SD wear. Retry backoff (5s/15s/60s,
then flat 60s) gated on `GsmManager::hasNetwork()`. Each of the 5 numbers
per event is an independent record, so one bad number never blocks the
other four.

### 5.7 LoRa packet schema + Give Way FSM (LoRaTransport, OvertakeManager)

```cpp
struct LoRaHeader {
  uint8_t  protoVersion;         // = 1
  uint8_t  msgType;              // CHECKPOINT_EVENT, OT_REQ, OT_DEV_ACK, OT_USER_ACK,
                                  // OT_USER_ACK_ACK, OT_CANCEL, BUSY, SESSION_END,
                                  // reserved: EMERGENCY, HELP
  uint16_t srcDeviceId;
  uint16_t dstDeviceId;          // 0xFFFF = broadcast
  uint16_t sessionId;
  uint16_t sequence;
  int32_t  correctedDistanceCm;  // signed, cm precision - avoids float-on-air determinism issues
};
// + msgType-specific payload (e.g. checkpoint label string; empty for OT_* control messages)
```
Integrity via the RA-02/SX1278 driver's own CRC (`LoRa.enableCrc()`) — a
software checksum would only be justified if fragmenting across multiple
packets, which this schema avoids by staying well under one packet's
payload limit.

FSM mirrors spec §15's table: requester `IDLE → REQUESTING → REQUEST_ACKED
→ GRANTED → COMPLETING → IDLE`; receiver `IDLE → DEVICE_RECEIVED →
DRIVER_GRANTED → IDLE`. `BUSY` returned by either side already in a
non-IDLE session with a different peer. 30s no-comms timeout (from last
successfully received message) unilaterally resets either side to IDLE.
Completion: `relativeDistance = peerCorrectedDistance - myCorrectedDistance`;
fire `SESSION_END` when its sign flips **and** `|relativeDistance|`
exceeds a hysteresis margin (recommendation: 5m, configurable) after the
flip, to avoid chatter at zero.

### 5.8 Wi-Fi / M8N settings / SD file-manager API (WebManager, ESP32-hosted)

- SoftAP `Tracking Device <device-id>`, open/no password (unchanged owner
  requirement); mDNS `<device-id>.local`.
- UI assets served from a LittleFS partition (editable without recompiling
  firmware).
- JSON API via `ESPAsyncWebServer` (chosen over the synchronous core
  `WebServer` specifically because a blocking `handleClient()` call would
  violate "web requests must not block GNSS parsing," §20 — matters since
  Key 2's 5s toggle can be pressed mid-race):
  - `GET /api/status`, `GET|POST /api/config` (M8N baud/rate/sentences
    included, applied via `GpsManager::applySettings()` after
    `ConfigManager::save()` validates)
  - `GET /api/sd/list`, `GET /api/sd/file`, `POST /api/sd/file` (small text
    edits; large files via multipart upload), `POST /api/sd/rename`,
    `DELETE /api/sd/file`, `POST /api/sd/mkdir`
  - Write endpoints return 409 while a race log is open (§5.2)

### 5.9 Libraries (whole system)

| Purpose | Library | Reason |
|---|---|---|
| OLED | `Adafruit_SH110X`+`GFX`+`BusIO` | Owner-mandated exact construction |
| NeoPixel matrix | `Adafruit_NeoPixel` | Owner-mandated exact declaration pattern |
| GNSS | `TinyGPSPlus` | Small, streaming `encode()` fits the non-blocking loop; kept separate from raw pass-through logging |
| SD | ESP32 core `SD`, `SdFat` swap-in as a **recommendation** pending Phase 8 write-latency benchmarking under sustained load | Both share `fs::FS`, low-risk to swap later |
| LoRa | `arduino-LoRa` over RadioHead | We already build our own session/ACK/retry layer (§15); RadioHead's addressed-datagram layer would duplicate it |
| SIM800L | Minimal custom AT-command driver, not TinyGSM | TinyGSM's `waitResponse()` blocks internally; a hand-rolled one-command-per-tick state machine is easier to *prove* non-blocking (tradeoff: more code to write) |
| Web server | `ESPAsyncWebServer`+`AsyncTCP` over core `WebServer` | Non-blocking by construction; see §5.8 |
| Web assets | `LittleFS` | Editable UI without recompiling |
| JSON | `ArduinoJson` | Config API + settings payloads |

### 5.10 Phased test plan

| Phase | Bench test | Pass criteria (ties to spec §22) |
|---|---|---|
| 1. Hardware foundation | Power on, watch OLED splash→init→data, press each key | Splash 3s exact, init messages match serial, DATA page never overlaps fields |
| 2. ReferenceMap/RouteMatcher | Feed a recorded NMEA log as ReferenceMap, verify segment/index output | Correct dedup point count, cumulative distance monotonic, index rebuild only on file change |
| 3. Geofence/logging | Simulate an NMEA replay crossing known geofences | Each point fires once, correct crossing time, log opens/closes per lifecycle rules incl. 20-min stop |
| 4. SIM800L | Bench SMS with signal pulled/restored | IMEI printed, pending queue persists and retries without duplicate delivered sends |
| 5. LoRa | Two boards, checkpoint event + range test | ACK/retry ~1s cadence, LoRa log has TX/RX+timestamp |
| 6. Give Way | Two boards simulate overtake at varying relative distance | Full FSM incl. BUSY, 30s timeout, sign-crossing completion with hysteresis |
| 7. Wi-Fi/WebManager | Laptop connects to AP, edits config, uses file manager | mDNS/IP shown, M8N settings apply live, file ops succeed, rejected mid-race |
| 8. Battery/reset/field test | ADC vs multimeter, power-cut mid-race, full recorded-track simulation | Voltage matches meter, reset recovery skips passed points and resumes logging |

### 5.11 Local time, and stale-GNSS handling

**Local time.** GNSS reports UTC. `AppConst::UTC_OFFSET_MINUTES_DEFAULT`
sets the default (UTC+5 = 300 minutes) and `AppConfig::utcOffsetMinutes`
makes it changeable at runtime (and later from the HTML settings page),
validated to ±14h. Stored in minutes so half/quarter-hour zones work.
`TimeUtil::applyUtcOffset()` does the conversion including correct date
rollover across month/year boundaries and leap years.

The offset applies **only to what a human reads** — the OLED crossing
time and the race-log filename. It is deliberately never applied to the
raw NMEA sentences written into the logs: those stay verbatim UTC,
because they are the authoritative record a replay tool has to be able to
trust.

**Stale GNSS.** TinyGPS++'s `isValid()` means "this field has been
populated at least once since boot" — it never goes back to false when
the fix is lost. Relied on alone, the DATA page would keep showing the
last known speed/satellites/accuracy indefinitely after the antenna is
unplugged, which reads as live data and is actively misleading. Every
GpsManager validity accessor therefore pairs `isValid()` with an `age()`
check against `AppConst::GNSS_FIX_MAX_AGE_MS` (3s), and each DATA-page
field is gated on its *own* freshness so they clear independently,
falling back to the `--` placeholder.

**What blanks on a dropout** (owner-confirmed): the live GNSS fields and
the logging indicator only — speed (Field 3), satellites (Field 9),
accuracy (Field 10), and the geofence label/distance (Fields 7/8, both
computed from live position), plus `L` (which goes out because a stale
speed counts as stopped, so writing pauses).

**What deliberately persists**, because it stays true rather than going
stale: the covered/corrected distance (Field 5, accumulated race state)
and the last captured crossing time (Field 2, a recorded past event), plus
battery voltage (not GNSS-derived). Distance accumulation resets its
previous-fix reference on a dropout, so reacquisition doesn't inject a
bogus straight-line jump.

Note the interaction with the 20-minute stop timeout: a stale fix reports
as 0 km/h, so a *continuous* 20-minute GNSS blackout would close the log
the same way a genuinely parked vehicle does. Any moment of valid movement
resets that timer, and during a total blackout there is no position data
worth logging anyway.

## 6. Laptop companion app (new scope, not part of the ESP32 firmware)

`companion/index.html` — a single self-contained static HTML/JS/CSS file,
opened directly in a browser (no server, no install, no network access of
any kind, no connection to the device). See `companion/README.md` for
usage. It deliberately re-implements the *exact same* NMEA epoch-dedup
algorithm as `ReferenceMapIndexer` (RMC/GGA grouped by shared UTC time
field, first-valid-coordinate wins, VTG ignored) so a track viewed/edited
here matches what the firmware would build from the same raw file.

Three tools in one page:
- **Race log viewer/replay** — loads a race log + optional GeoFencing.txt,
  plots the track, computes each geofence's closest-approach match against
  the recorded track, and provides a scrub/playback slider with live
  time/speed/cumulative-distance/sats/HDOP readout.
- **GeoFencing.txt editor** — an editable table (load/add/remove rows,
  validate against the same rules as `GeoFenceManager`, click-on-track to
  add a point snapped to the nearest track sample with its cumulative
  distance auto-filled) and exports in the exact confirmed
  `latitude,longitude,distance_m,label` format.
- **ReferenceMap trim/merge** — loads one or more raw NMEA captures, lets
  you select a sub-range on each (via a dual-handle slider over the parsed
  track) and assemble an ordered output sequence, then exports a combined
  `ReferenceMap.log`. Only ever slices/concatenates genuine captured
  lines byte-for-byte — never synthesizes a sentence — keeping the
  raw-NMEA-authoritative principle intact even in an editor.

Verified end-to-end with a real headless-Chromium run (dedup count,
crossing detection, exact-format export, and byte-for-byte trim were all
checked against a synthetic fixture — not just eyeballed).

## 7. Remaining genuinely open items

1. Literal NMEA identifier for "GNGTV" (almost certainly VTG) — doesn't
   block anything since VTG carries no position.
2. RA-02 regional legal TX power / BW / SF / CR limits for your deployment
   country (433 MHz center is confirmed).
3. No RTC on the pin map — race-log filename fallback before first GNSS fix
   (§5.5) is a recommendation pending your confirmation.
4. GNSS accuracy source is an engineering approximation (`HDOP × 5m`),
   flagged not asked, since the enabled sentence set has no native
   accuracy field.
5. Your actual GeoFencing.txt / ReferenceMap.log, to validate the parsers
   (firmware and companion app both) against real data instead of only the
   confirmed example rows.
