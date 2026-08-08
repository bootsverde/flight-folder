# Flight Folder

ESP32-S3 flight-instrument HUD firmware (attitude/GPS/altitude display for a
Jeep), plus a Raspberry Pi companion display. Two board targets, each a
self-contained PlatformIO project:

- **[7inch/](7inch/)** — Waveshare ESP32-S3-Touch-LCD-7B, 1024x600 RGB panel.
- **[4.3inch/](4.3inch/)** — Waveshare ESP32-S3-Touch-LCD-4.3B, 800x480 RGB panel.

Open either subfolder directly as its own PlatformIO project (own
`platformio.ini` + `src/`). The two share no code — each has its own copy of
`main.cpp` and `jeep_bitmap.h`, since the HUD layout is hand-tuned in pixel
coordinates per display resolution.

`pi-hud/` is a separate Raspberry Pi Python HUD (pygame), independent of
which ESP32 board is in use.

## 4.3inch notes (new, unverified on hardware)

- Display bus timings and RGB/touch pin mapping came from Waveshare's own
  4.3B reference code — same carrier-board pinout as the 7B, just different
  resolution/timing constants.
- All HUD layout coordinates were proportionally rescaled from the 7"
  version (800/1024 x-scale, 480/600 y-scale), not redrawn from scratch.
  Expect some things to want manual nudging once you see it on the real
  panel.
- Touch calibration raw range (`readTouch()`) is back-derived math, not
  measured on this panel — recalibrate against `ts.points[0]` raw values if
  taps land off.
- The jeep bitmap icon is unscaled (same pixel art as the 7" build), so it
  will look proportionally a bit larger on the smaller screen.
- Both `platformio.ini` files default to `COM6` — update one if you have
  both boards connected at once.
