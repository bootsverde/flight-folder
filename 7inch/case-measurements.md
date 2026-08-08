# ESP32-S3-Touch-LCD-7B — Case Panel Measurements

Standalone box, flat-panel construction (acrylic or plywood, hand tools — no 3D printer/laser).
Measurements below are from the physical board, taken 2026-08-03. Values marked **[confirm]** need
a follow-up check before cutting anything.

## Board / display envelope
- Display cutout: **165.6mm (W) x 99mm (H)** — corrected 2026-08-03, landscape (wider than tall).
  Supersedes an earlier 192.94 x 110.67mm reading.
- Overall depth needed behind front panel: **32.25mm** (board + connectors + display ribbon clearance)

## Front/back panel — OUTER SIZE CONFIRMED
Bezel margins around the display cutout: bottom 5mm, top 8mm, right 12mm, left 15.4mm.

- **Panel outer size: 193mm (W) x 112mm (H)** = (15.4 + 165.6 + 12) x (5 + 99 + 8)
- Cutout position within the panel: **15.4mm from left, 5mm from bottom**

## Reference (from Waveshare product listing, not directly measured — verify against your board)
- Overall PCB size: 165.72mm x 126.20mm
- Mounting holes: 4x M3, 4x M2.5 (two different patterns — exact XY positions not yet confirmed)

## Power / external connectors
- **Power plan:** panel-mount USB-C bulkhead fitting on the back panel, wired externally to a
  12V-to-USB-C buck converter for vehicle/boat 12V supply. Inside the case, a short USB-C jumper
  cable runs from the bulkhead fitting to the board's onboard USB-C port. Because a cable bridges
  the gap, the bulkhead's panel position doesn't need to line up precisely with the board's
  onboard USB-C location — placement is flexible, pick wherever's convenient on the back panel.
- **UART1:** accessory-only header (GPS/debug device power+data output), NOT used for external
  power input. Vertical span from the bottom edge: **top at 57.69mm**, **bottom at 48.71mm**
  (connector height ~8.98mm). X position (from left edge) not yet measured — only needed if
  something plugs into it that requires panel access; otherwise no cutout needed for it.

## Mounting / attachment holes
Rectangular 4-hole pattern, **124.36mm x 66.28mm**, anchored with the bottom row at
**23.50mm from the bottom edge**. All four hole coordinates (X from left, Y from bottom):

| Hole | X (from left) | Y (from bottom) |
|------|---------------|------------------|
| Bottom-left  | 36.44mm  | 23.50mm |
| Bottom-right | 160.80mm | 23.50mm |
| Top-left     | 36.44mm  | 89.78mm |
| Top-right    | 160.80mm | 89.78mm |

This is the **4x M3** hole set — only these 4 are used for mounting (M2.5 holes not needed for
this case). These coordinates are board-referenced, not panel-referenced — user will mark and
drill the holes directly during assembly (board mounted behind the cutout, holes marked in place)
rather than needing pre-computed panel coordinates. No further translation needed.

## Still needed
- [ ] Any other external connectors/buttons (power switch, SD card, speaker, etc.)
- [ ] Bulkhead USB-C fitting's own cutout dimensions (panel hole size for the fitting itself)
