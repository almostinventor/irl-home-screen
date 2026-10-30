# Software setup

This covers getting the firmware and the projector app running. How you build the shelf, the
rail and the pushers is up to you; the printed parts are in `hardware/stl/` and the parts
list is in [BOM.md](BOM.md).

The code is AI-generated, and the wiring is at your own risk. I am not an electrical engineer.

## Pin list

LOLIN S2 Mini, defaults in `irl_home_screen.ino`:

```
GPIO40   TMC2209 STEP
GPIO38   TMC2209 DIR
GPIO10   TMC2209 EN (active low)
GPIO18   TMC2209 PDN_UART
GPIO17   TMC2209 PDN_UART, through a 1k resistor
GPIO33   servo A signal
GPIO34   servo B signal
GPIO35   rail endstop, NC switch to GND
GPIO4    lever, NO switch to GND
```

TMC2209 MS1 and MS2 to GND (UART address 0). Common ground. Motor current is set in the
firmware, not with the VREF pot.

About the 1k resistor: the TMC2209 talks over a single wire (PDN_UART) in both directions,
while the ESP32 has separate send (TX) and receive (RX) pins. RX goes straight to PDN_UART.
TX goes through the 1k resistor, so when the driver answers on the same wire it can pull the
line against the ESP32's TX pin without the two shorting each other. If `T` over serial
reports FAIL, check this resistor and that the motor supply is on (the driver does not
answer without it).

## Firmware

1. Arduino IDE with the **esp32** board package **2.0.x** (3.x will not compile) and the
   libraries **TMCStepper**, **AccelStepper** and **ESP32Servo**.
2. Copy `firmware/irl_home_screen/secrets.example.h` to `secrets.h` and add your WiFi
   (2.4 GHz only).
3. Set `STATIC_IP` and `GATEWAY_IP` in `irl_home_screen.ino` for your network.
4. Board: **LOLIN S2 Mini**. Hold BOOT, tap RST, upload, then press RST.

The board homes the rail every time it powers up. Over USB serial (115200), `?` prints the
status and `T` checks the driver connection.

## Rail

Open `http://<board-ip>/` in a browser.

1. **HOME** the rail. If it drives the wrong way, flip `HOME_DIRECTION` in the firmware.
2. Jog to the end of the travel and press **SET RAIL END HERE**.
3. For each slot: pick the pusher arm (A or B) first, jog until it lines up with the case,
   press **SAVE**, then try **EJECT**.

## Projector app

1. Install Python 3, then `pip install -r app/requirements.txt`.
2. In `app/projector.html`, set `ESP32_URL` to the board's IP and edit `SLOTS` to match your
   shelf. The same list of slots is `SLOT_NAMES` in the firmware; keep them in sync.
3. Optional: put logo images in `app/logos/` (see the README there).
4. Run `python app/main.py` with the projector as a display.

## Calibration

Click on the projected image while looking at the shelf. The banner at the top left says what
to click; `Backspace` undoes.

- `C` highlight boxes: top and bottom edge once, then left and right edge of each slot.
- `B` name-tag box: left, right, top, bottom.
- `R` rail positions from the projector screen (same as the dashboard).
- `E` extra decorative boxes.

## Keys

| Key | |
| --- | --- |
| Click, `Space`, `Enter` | Push out the highlighted game; the next input puts it back |
| `Esc` | Reset (in a calibration mode: leave it) |
| `D` | Outro light show |
| `K` | Classic gold look |
| `T` | Test mode |
| `M` | Settings menu |
| `F` | Fullscreen |

Pulling the lever, or clicking its slot, starts the roulette. Clicking the name-tag area opens
search.
