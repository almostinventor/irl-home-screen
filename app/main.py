"""
IRL Home Screen - projector app.

Opens projector.html fullscreen in a pywebview window. Put the window on the
projector's display (or make the projector your main display) before starting.

Run:
    pip install -r requirements.txt
    python main.py
"""

import os

import webview

HTML_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "projector.html")

# pywebview defaults to private_mode=True, which hands the browser engine a throwaway
# profile that is wiped on exit -- every launch would start with an empty localStorage
# and all the calibration (highlight boxes, name tag box, extra boxes) would have to be
# redrawn. Persisting the profile to a fixed per-user folder is what makes calibration
# survive a restart.
if os.name == "nt":
    _base = os.environ.get("LOCALAPPDATA") or os.path.expanduser("~")
else:
    _base = os.environ.get("XDG_DATA_HOME") or os.path.join(os.path.expanduser("~"), ".local", "share")
STORAGE_PATH = os.path.join(_base, "IRLHomeScreen")


def main():
    os.makedirs(STORAGE_PATH, exist_ok=True)
    webview.create_window("IRL Home Screen", HTML_PATH, fullscreen=True)
    webview.start(private_mode=False, storage_path=STORAGE_PATH)


if __name__ == "__main__":
    main()
