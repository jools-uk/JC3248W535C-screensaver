# JC3248W535C Mandelbrot Screensaver

A demonstration of the **JC3248W535C** display module on an ESP32-S3, implementing an animated Mandelbrot fractal screensaver with touch interaction.

The primary goal of this project is to serve as a working, well-documented example of how to drive the AXS15231B QSPI display controller and AXS15231B touch IC directly from ESP-IDF — without relying on any framework (no Arduino, no LVGL, no AXS15231B managed component).

**Deliberately no LVGL or graphics library.** The entire display pipeline — QSPI command framing, window addressing, DMA strip transfers, software rotation, colour conversion — is implemented by hand. The intent is to show the raw wire protocol in action so the code can serve as a reference for anyone needing to understand exactly what bytes go over the bus and why. Higher-level libraries such as LVGL are perfectly valid for production use, but they hide these details behind abstraction layers that make debugging hardware bringup significantly harder.

---

## Hardware

| Item | Detail |
|------|--------|
| Module | JC3248W535C |
| MCU | ESP32-S3 |
| Display controller | AXS15231B (QSPI, 4-wire) |
| Display panel | 320×480 portrait (native) |
| Touch IC | AXS15231B (in-cell, I2C) |
| PSRAM | 8 MB octal (used for the fractal index buffer) |

---

## Display driver

The display is driven via **Quad-SPI** using the ESP-IDF `esp_lcd_panel_io` layer. The QSPI command format — derived from the ESPHome `qspi_dbi` component, which was confirmed working on this exact hardware — is:

- `spi_mode = 0` (CPOL=0/CPHA=0)
- `lcd_cmd_bits = 32`: each command header is four bytes sent in 1-bit SPI: `[opcode][0x00][cmd][0x00]`
- Pixel data follows in 4-bit QSPI mode
- Opcode `0x02` = write command/parameter; opcode `0x32` = write colour data (RAMWR)
- RASET is sent before CASET (ESPHome order)

The init sequence matches the ESPHome `axs15231` model exactly (vendor unlock, `0xC1{0x33}`, vendor lock, INVOFF, MADCTL, COLMOD, BRIGHTNESS, DISPON).

---

## Display rotation — why software, not hardware

The AXS15231B datasheet describes hardware address reordering via the MADCTL register (`0x36`), specifically the `MV` bit (`cr_rot9_en`) for XY-exchange (90°/270° landscape) and `MX`/`MY` bits for mirroring. In principle this should allow the display controller to handle landscape orientation transparently.

**In practice, the address reordering does not work on this module.** Setting any combination of MV/MX/MY produces only a narrow strip of pixels at one edge of the display regardless of the pixel data sent. This matches anecdotal reports from other developers who have used the same module.

The most likely explanation is that the GRAM address remapping logic described in the datasheet was never actually implemented in the production silicon — the register is accepted without error but has no effect. The panel always scans in portrait row-major order (X/CASET increments first, 320 columns wide, 480 rows tall) regardless of MADCTL. This is consistent with a cost or complexity reduction made during final tapeout, where the feature was documented in the datasheet but omitted from the die.

### Software rotation

All orientation handling is done in the **flush functions** in `display.c` and `fractal.c`. A single `DISP_ROTATE` define (set in `config.h`) selects 0°, 90°, 180°, or 270°. The logical framebuffer is always in the target orientation (e.g. 480×320 landscape for `DISP_ROTATE=90`); the flush loop transposes pixels into physical portrait strips before DMA-ing them to the display.

For 90° CW rotation the transform is:

```
portrait pixel (col=c, portrait_row=y0+r)  ←  landscape pixel (lx = DISP_H-1-y0-r, ly = c)
```

This is implemented with a backward-sequential PSRAM read (stride −1 along a landscape row) which keeps cache behaviour reasonable.

The strip buffer lives in DMA-capable internal SRAM (`MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL`); the large fractal index buffer lives in PSRAM.

---

## Touch coordinate mapping

The AXS15231B touch IC reports raw coordinates via I2C using an 8-byte protocol (write `{0xb5, 0xab, 0xa5, 0x5a, 0x00, 0x00, 0x00, 0x08}`, read 8 bytes). The raw axes have both directions physically reversed relative to the portrait display — `raw_x=0` is at the right edge and `raw_y=0` is at the bottom. This is corrected in `touch_read()` in `touch.c` with per-rotation mappings:

| `DISP_ROTATE` | lx | ly |
|---|---|---|
| `0` (portrait) | `(LCD_W−1) − raw_x` | `(LCD_H−1) − raw_y` |
| `90` (landscape CW) | `raw_y` | `(LCD_H−1) − raw_x` |
| `180` (portrait flipped) | `raw_x` | `raw_y` |
| `270` (landscape CCW) | `(LCD_W−1) − raw_y` | `raw_x` |

The 180° case is a no-op because the two physical IC reversals cancel each other out.

---

## Fractal screensaver

### Coordinate system

The fractal uses a floating-point view defined by three values:

- `view_cx`, `view_cy` — the fractal coordinate at the centre of the screen
- `view_hw` — the half-width of the view in fractal units (the x-axis spans `[cx−hw, cx+hw]`)

The y half-height is derived from `hw` and the display aspect ratio: `hh = hw × (LCD_H / LCD_W)`.

The Mandelbrot set occupies roughly `x ∈ [−2.5, 1.0]`, `y ∈ [−1.25, 1.25]`. The "home" view (`HOME_CX=−0.5`, `HOME_CY=0`, `HOME_HW=10`) shows the full set small in a large surrounding area, providing room to explore.

### Rendering

Each frame renders into a PSRAM index buffer (`index_buf`, 480×320 × 2 bytes). Each pixel stores a smoothed palette index (not a raw colour) so the palette can be cycled each frame without re-rendering — the palette animation is essentially free.

Smooth colouring uses the standard escape-radius formula:

```
t = iter + 1 − log2(log2(|z|²) / 2)
```

The main cardioid and period-2 bulb are detected analytically (early-out) to avoid the full iteration loop for a large fraction of pixels.

### Auto-zoom

After `ZOOM_FRAMES` frames of palette animation, the view automatically zooms in:

1. The **deepest boundary pixel** — the escaped pixel with the highest iteration count in the current render — is tracked during rendering and stored as `zoom_target`. This pixel sits closest to the set boundary and tends to be a visually interesting feature.
2. The view centre moves to `zoom_target` and `view_hw` is multiplied by `ZOOM_STEP` (0.90 — a 10% zoom per step).
3. The fractal re-renders at the new view and the cycle repeats.

After a touch-triggered reset, the centre is held fixed for `tap_anchor_steps` (5) zoom steps while `hw` shrinks. This prevents the first auto-zoom step from jumping to a meaningless `zoom_target` computed from the large initial view; by step 5 the view is small enough that `zoom_target` reliably points to a genuine feature of interest.

When floating-point precision is exhausted (`view_hw < ZOOM_MIN = 1×10⁻⁶`), the view resets to the home position.

### Touch to reset

A tap anywhere on the screen:

1. Maps the touch pixel through the **fixed home view frame** (`HOME_CX/CY/HW`) to find a consistent fractal coordinate regardless of the current zoom depth.
2. Sets that coordinate as the new `view_cx/cy` — so tapping the left side of the screen always zooms into the left side of the set, tapping the right zooms into the right, etc.
3. Resets `view_hw = HOME_HW` (full zoom-out).
4. Anchors the centre for 5 steps before the auto-zoom resumes chasing `zoom_target`.

A 60 ms debounce (3 consecutive no-touch polls at 20 ms each) prevents spurious re-triggers from the touch IC dropping samples mid-press.

---

## Build

Normal fractal screensaver:
```bash
idf.py build flash monitor
```

Standalone display driver test (R/G/B stripe pattern, no fractal or touch):
```bash
idf.py build -DDISPLAY_TEST=ON flash monitor
```

Switch back:
```bash
idf.py build -DDISPLAY_TEST=OFF
```

> **Note:** CMake caches the `DISPLAY_TEST` flag. If switching targets doesn't take effect, run `idf.py fullclean` first.

### Orientation

Edit `config.h` to change the display rotation:

```c
#define DISP_ROTATE  90   // 0=portrait, 90=landscape CW, 180, 270
```

Touch coordinate mapping updates automatically to match.
