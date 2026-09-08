# Desert Race Companion (laptop tool)

A standalone local web page — **not** part of the ESP32 firmware, and it
never connects to the device. It runs entirely inside your browser tab.

## How to open it

No install, no server. Just double-click `index.html` (or right-click →
Open with → your browser). Works in Chrome, Edge, or Firefox.

## What it does

**Race Log Viewer / Replay** — load a race log (raw NMEA, copied off the
device's SD card) and optionally a `GeoFencing.txt`. It plots the track,
matches each geofence point against the recorded track by closest
approach, and gives you a scrub/playback slider with live time, speed,
cumulative distance, satellite count, and HDOP.

**GeoFencing.txt Editor** — load an existing `GeoFencing.txt`, edit rows
in a table, or load a reference track and click on it to add a point
(snapped to the nearest recorded position, with its distance-from-start
filled in automatically). Validates the same rules the firmware does
(latitude/longitude range, non-negative distance, non-empty label) before
you export. Exports in the exact confirmed format:
`latitude,longitude,distance_from_start_m,label` — no header.

**ReferenceMap Trim / Merge** — load one or more raw NMEA captures (e.g.
a recon lap that has a bad start or a gap you want to remove, or two
partial captures to stitch into one). Pick a sub-range on each with a
slider, queue up the pieces you want in order, and export a combined
`ReferenceMap.log`. This only ever copies real captured lines byte-for-byte
— it never invents or fabricates a sentence, so the exported file stays
just as trustworthy as a raw device recording.

## Why it's built this way

It re-implements the exact same NMEA dedup logic the firmware's
`ReferenceMapIndexer` uses (grouping GNRMC/GNGGA sentences into physical
fixes by their shared UTC time field, ignoring VTG), so what you see here
matches what the device would build from the same file.

Everything happens locally in your browser — nothing you load is uploaded
anywhere, and there's no dependency on Wi-Fi, the device, or an internet
connection.

## Known limitations

- Very large ReferenceMap files (tens of MB) will parse more slowly here
  than on the device, since this re-parses the whole file in memory each
  time you load it — fine for a single recon lap, not intended for
  editing the full ~250 km file in one sitting.
- Geofence labels containing literal quote characters may render oddly in
  the editable table (a cosmetic edge case, not a data-corruption risk).
