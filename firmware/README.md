# Firmware

`irl_home_screen/irl_home_screen.ino` runs on a LOLIN S2 Mini (ESP32-S2) and controls the rail, the two
pusher servos, the endstop and the roulette lever. It serves an HTTP API for the projector app
and a setup dashboard at `http://<board-ip>/`.

Before building, copy `irl_home_screen/secrets.example.h` to `irl_home_screen/secrets.h` and fill in your
WiFi details, then set the static IP near the top of `irl_home_screen.ino`.

Build requirements:

- Board package: **esp32 by Espressif Systems, 2.0.x** (2.0.17 tested). Version 3.x changed
  the hardware timer API and will not compile this sketch.
- Board: **LOLIN S2 Mini**
- Libraries: **TMCStepper** (teemuatlut), **AccelStepper**, **ESP32Servo**

Command-line build with `arduino-cli`:

```
arduino-cli compile --fqbn esp32:esp32:lolin_s2_mini firmware/irl_home_screen
arduino-cli upload  --fqbn esp32:esp32:lolin_s2_mini -p COM3 firmware/irl_home_screen
```

To upload, put the board in its bootloader first (hold BOOT, tap RST, release BOOT), and press
RST afterwards to run the new firmware.

Setup steps are in [docs/SETUP.md](../docs/SETUP.md). The HTTP endpoints and serial commands
are listed in the comment at the top of the sketch.
