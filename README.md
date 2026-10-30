# IRL Home Screen

A shelf of game cases that picks a game for you.

A projector shines down onto a real shelf of game cases and draws a glowing highlight on
whichever case you point at. Click it, and a carriage on a rail behind the shelf drives to that
case and a small servo arm pushes it forward so you can grab it. The projected name tag shows the
game's logo, a few stats and a short description while it is out. Pull the lever at the end of
the shelf and a roulette of lights spins across the cases and picks one at random.

By **AlmostInventor**.

> **All of the code in this repository is AI-generated.** The firmware, the projector app, the
> tools and the documentation were written with an AI coding assistant (Claude) and then tested
> on the real shelf. Read it, test it on your own hardware, and treat it accordingly.

> **Build and wire this at your own risk.** I am not an electrical engineer. The pin list
> and instructions show how the reference shelf was put together, not a certified or verified
> design. This project uses mains-powered supplies, motor currents and parts that can get hot,
> so check everything yourself, follow the instructions that come with your power supplies and
> components, and ask someone qualified if you are unsure. I take no responsibility for damage
> or injury resulting from building or using it.

## Features

- **Point and pick:** any mouse, or a motion remote used as a pointer, highlights the case it is
  over, and a click pushes that case out.
- **Physical push-out:** one carriage on a rail, two servo pusher arms (one per side of the
  carriage), positions taught slot by slot.
- **Roulette lever:** a real lever on the shelf starts a spin. Four lights race around the
  shelf, drop out one by one, and the last one lands on a random game.
- **Info card** in the name-tag area while a case is out: logo, star rating, hours played, max
  players and a one-line description (all editable).
- **Calibration built in:** you click the edges of each case on the wall, and everything is saved.
- **Extras:** an outro light show, a classic gold-glow look, synthesized sound effects, a
  one-key reset, and a web dashboard on the controller for setting up the rail.

## How it works

```
 Laptop (projector app) --WiFi / HTTP--> ESP32-S2 controller --> TMC2209 --> rail stepper
        |                                    |--> pusher servo A / B
        +-- projector draws the highlights   |--- endstop (homing)
                                             +--- roulette lever
```

- `app/` is the projector app: one HTML page (`projector.html`) shown fullscreen on the
  projector by a tiny Python launcher (`main.py`). It talks to the controller over plain HTTP.
- `firmware/irl_home_screen/` is the ESP32-S2 sketch: rail motion, homing, the two pushers, the
  lever, a web dashboard, and the HTTP API the app uses.
- `hardware/stl/` holds the 3D-printed parts.
- `docs/` has the parts list and the setup guide.

## Setup

See **[docs/SETUP.md](docs/SETUP.md)** for the pin list, flashing the firmware, setting up the
rail and running the projector app. The parts are listed in [docs/BOM.md](docs/BOM.md) and the
printable models are in `hardware/stl/`.

## What is not included

No game artwork, logos or other media are included. Put your own logo images in `app/logos/`
(see the README there); any game without one simply shows its title as text. The game list in
the code is an example, so replace it with your own shelf.

## License

- Code (`app/`, `firmware/`, `tools/`): [MIT](LICENSE)
- 3D models and documentation (`hardware/`, `docs/`): [CC BY 4.0](hardware/LICENSE.md)

Game titles in the example configuration are trademarks of their respective owners. This is an
independent hobby project, not affiliated with or endorsed by any game company.
