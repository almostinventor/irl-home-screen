"""Read the IRL Home Screen controller's status over USB serial. Read-only.

Sends only '?' (status), 'T' (TMC2209 UART check) and 'K' (slot table). Every
other serial command moves the carriage or fires a pusher, so this script never
sends them.

    pip install pyserial
    python tools/serial_status.py COM3        (Windows)
    python tools/serial_status.py /dev/ttyACM0 (Linux)

The LOLIN S2 Mini uses native USB, so two things are worth knowing:

1. DTR must be asserted. The board only sends serial output while a terminal
   holds DTR, so a monitor that opens the port with DTR low sees nothing at all
   from a perfectly healthy board. This script asserts it.
2. The port can change number after the board resets or enters the bootloader.
   If the script cannot open the port, check which one the board is on now.
"""
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is not installed: python -m pip install pyserial")

args = [a for a in sys.argv[1:] if not a.startswith("-")]
if not args:
    sys.exit("usage: python tools/serial_status.py <port>   e.g. COM3 or /dev/ttyACM0")
PORT = args[0]

COMMANDS = [("?", "status"), ("T", "TMC2209 UART check"), ("K", "slot table")]

s = serial.Serial()
s.port = PORT
s.baudrate = 115200
s.timeout = 0.2
s.dtr = True
s.rts = False
try:
    s.open()
except serial.SerialException as exc:
    sys.exit("could not open %s: %s" % (PORT, exc))

time.sleep(0.3)
s.reset_input_buffer()


def drain(seconds, label):
    print("--- %s ---" % label)
    end = time.time() + seconds
    got = b""
    while time.time() < end:
        chunk = s.read(4096)
        if chunk:
            got += chunk
            end = time.time() + 0.6  # keep reading while it is still talking
    text = got.decode("utf-8", "replace").strip()
    print(text if text else "(nothing)")


for cmd, label in COMMANDS:
    s.write((cmd + "\n").encode())
    s.flush()
    drain(2.0, "'%s' %s" % (cmd, label))

s.close()
