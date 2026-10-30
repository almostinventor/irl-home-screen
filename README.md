# IRL Home Screen

YouTube video link coming.

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
