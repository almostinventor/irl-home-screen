/*
  IRL Home Screen - rail pusher controller
  Board: LOLIN S2 Mini (ESP32-S2)

  One carriage on a rail drives to whichever shelf slot is selected, and one of
  TWO pusher servos on the carriage pushes that game case forward. A randomiser
  lever on the shelf starts a roulette spin in the projector app.

  Before building: copy secrets.example.h to secrets.h (next to this file) and
  fill in your WiFi details. secrets.h is git-ignored.

  Wiring
  -------------------------------------------------------------------
    TMC2209        LOLIN S2 Mini / supply
    -------        ----------------------
    STEP    ->     GPIO40
    DIR     ->     GPIO38
    EN      ->     GPIO10   (active LOW: LOW = driver enabled)
    VIO     ->     3V3
    GND     ->     GND (logic) and motor PSU - (common ground required)
    VM      ->     motor PSU +  (check your module's rating, typically 5.5-28 V)
    PDN_UART ->    GPIO18 directly, and GPIO17 through a 1 kOhm resistor
                   (single-wire UART: the resistor joins TX onto the RX line)
    MS1, MS2 ->    GND (UART address 0)
    1A/1B, 2A/2B -> stepper coils

    Pusher servo A ->  GPIO33   (signal; 5 V + GND from the servo supply)
    Pusher servo B ->  GPIO34
    Endstop        ->  GPIO35   (normally-closed switch to GND - see ENDSTOP_ACTIVE_HIGH)
    Randomiser lever -> GPIO4   (normally-open switch to GND)

  Libraries (Arduino Library Manager): "TMCStepper" (teemuatlut),
  "AccelStepper", "ESP32Servo". Board package: esp32 by Espressif, 2.0.x -
  the 3.x core changed the hardware-timer API this sketch uses.

  ---------------------------------------------------------------------------
  The two pushers
  ---------------------------------------------------------------------------
  Which arm ejects a slot is stored per slot and taught alongside its rail
  position. /api/split is a bulk fill (slots below N to A, the rest to B) that
  you can then adjust slot by slot.

  The arms' offset along the carriage is NOT modelled anywhere. Each slot's
  carriage position is taught by hand with the arm that will serve it, so the
  offset is baked into that slot's saved step count. That is also why changing
  a slot's arm clears its taught position: re-teach it afterwards.

  ---------------------------------------------------------------------------
  The logical axis
  ---------------------------------------------------------------------------
  Positions the outside world sees always run 0..railMaxSteps, where 0 is the
  endstop and positive travel moves AWAY from it. The physical direction that
  corresponds to is HOME_DIRECTION, and AXIS_SIGN flips between the two. Every
  move goes through railMoveTo()/railPosition(), so flipping HOME_DIRECTION
  flips the working range with it.

  ---------------------------------------------------------------------------
  Endpoints - shelf app (app/projector.html)
  ---------------------------------------------------------------------------
    GET /api/status             -> full status json
    GET /api/select?game=N[&hold=1] -> drive to slot N and push it out
                                   (hold=1: leave it out until /api/retract)
    GET /api/retract            -> pull a held case back in
    GET /api/goto?game=N        -> drive to slot N, no push
    GET /api/home               -> run the homing sequence
    GET /api/testpush[?which=a|b] -> fire a pusher where the carriage stands
    GET /api/stop               -> abort motion and any in-flight select
    GET /api/slots              -> [{"i","name","step","set","pusher"}, ...]
    GET /api/setslot?game=N[&steps=M][&pusher=a|b] -> teach slot N (default: current position)
    GET /api/clearslot?game=N   -> forget slot N
    GET /api/resetslots         -> forget every slot
    GET /api/slotpusher?game=N&pusher=a|b -> assign one slot to an arm
    GET /api/split?slot=N       -> bulk fill: 0..N-1 to A, the rest to B
    GET /api/speed?stepspersec=N   -> cruise speed, persisted
    GET /api/accel?stepspersec2=N  -> acceleration, persisted
    GET /spin-status            -> {"pressed":bool} latched randomiser lever

  Endpoints - bench dashboard (open http://<board-ip>/ in a browser)
  ---------------------------------------------------------------------------
    GET /                       -> dashboard (motion, tuning, homing, slot calibration)
    GET /status                 -> alias of /api/status
    GET /jog?steps=N            -> relative move, microsteps (+/-)
    GET /moveto?pos=N           -> absolute move, logical steps
    GET /zero                   -> redefine current position as 0
    GET /setend                 -> redefine current position as the far end, persisted
    GET /home                   -> alias of /api/home
    GET /speed?stepspersec=N    -> cruise speed (10-20000)
    GET /accel?stepspersec2=N   -> acceleration (10-100000)
    GET /enable?on=1|0          -> engage/disengage the driver (0 = push it by hand)
    GET /current?ma=N           -> RMS run current, mA (clamped to CURRENT_MAX_MA)
    GET /microsteps?n=N         -> 1,2,4,8,16,32,64,128,256
    GET /stealth?on=1|0         -> 1 = StealthChop (quiet), 0 = SpreadCycle
    GET /servo/a?angle=N        -> pusher A, direct instant angle (0-180)
    GET /servo/b?angle=N        -> pusher B, direct instant angle (0-180)
    GET /servo/a/push           -> pusher A full push cycle
    GET /servo/b/push           -> pusher B full push cycle
    GET /seq/add?slot=N         -> append slot N to the demo sequence
    GET /seq/clear              -> empty the sequence
    GET /seq/play?dwell=N&loop=0|1&push=0|1 -> play it, optionally pushing at each stop
    GET /seq/stop               -> stop playback
    GET /seq/status             -> sequence json

  Serial commands (115200 baud), one per line:
    j <steps>  relative jog          m <pos>   absolute move
    z          zero here             x         set rail end here
    h          home                  g <slot>  goto slot
    G <slot>   select (goto + push)  k <slot>  teach slot from current position
    K          list the slot table   s <speed> cruise speed
    A <slot>   assign slot to arm A  B <slot>  assign slot to arm B
    a <accel>  acceleration          e 0|1     enable/disable driver
    c <mA>     run current           u <N>     microsteps
    t 0|1      stealthchop           p/q <ang> servo A/B direct angle
    P / Q      push servo A / B      ?         status
    T          TMC2209 UART check

  ---------------------------------------------------------------------------
  Open-loop: nothing except the endstop confirms the motor kept up with the
  pulses. If it stalls, the tracked position drifts until the next home. Raise
  speed/accel gradually.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <TMCStepper.h>
#include <Preferences.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <esp_system.h>   // esp_reset_reason() - see the WiFi black box

// WIFI_SSID, WIFI_PASSWORD and (optionally) WIFI_BSSID. Copy secrets.example.h
// to secrets.h and fill it in; secrets.h is git-ignored.
#include "secrets.h"

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------
// Declared up here on purpose: the Arduino build injects auto-generated function
// prototypes immediately before the first function definition in the sketch.
// Anything appearing in a function signature - ServoPusher and LatchedInput both
// do - must be declared above that injection point, or the generated prototypes
// reference an unknown type and the build dies with "variable or field ...
// declared void".

enum PushPhase { PUSH_IDLE, PUSH_DOWN, PUSH_WAIT, PUSH_UP };

struct ServoPusher {
  Servo* servo;
  int* angle;
  int restAngle;   // idle/retracted position
  int pushAngle;   // extended position; rest->push sets this pusher's rotation direction
  PushPhase phase;
  float currentAngleF;
  unsigned long lastStepMs;
  unsigned long waitStart;
  // Stay extended until released instead of retracting after SERVO_PUSH_WAIT_MS.
  // Set per push by the select state machine, cleared by releasePushers().
  bool hold;

  // Explicit constructor rather than brace-init with default member initializers:
  // the ESP32 core 2.x builds with -std=gnu++11, where in-class initializers stop
  // a struct from being an aggregate, so `ServoPusher p = { &servo, &angle };`
  // fails to compile.
  ServoPusher(Servo* s, int* a, int rest, int push)
    : servo(s), angle(a), restAngle(rest), pushAngle(push),
      phase(PUSH_IDLE), currentAngleF((float)rest), lastStepMs(0), waitStart(0),
      hold(false) {}
};

// A momentary switch read as a latched event rather than a level.
//
// Two failures this deliberately avoids:
//   - `settled` is tracked separately from `raw`. A debounce that resets its
//     timer and compares against the previous *reading* in the same iteration
//     always discards the edge, so the button never fires.
//   - The event latches until it is collected. The app polls every 200 ms; a
//     pull between two polls would vanish on a level read, and a slow release
//     would report one pull as several spins.
struct LatchedInput {
  int pin;
  bool activeLow;
  bool raw;             // last instantaneous reading
  bool settled;         // reading that has held steady past the debounce window
  unsigned long lastChangeMs;
  bool latched;         // an active edge happened and nobody has collected it yet
  unsigned long lastEventMs;

  LatchedInput(int p, bool aLow)
    : pin(p), activeLow(aLow), raw(false), settled(false),
      lastChangeMs(0), latched(false), lastEventMs(0) {}
};

// Drive-then-eject runs as a state machine so the web server keeps answering
// during the several seconds a full ejection takes.
enum SelectPhase { SEL_IDLE, SEL_TRAVEL, SEL_SETTLE, SEL_PUSH };

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

// Static IP. app/projector.html talks to the board at a fixed address
// (ESP32_URL), so it cannot be left to DHCP - the shelf would lose the board on
// every lease change. Pick a free address on YOUR network, outside the router's
// DHCP range, and put the same address in projector.html.
// The ESP32-S2 has no 5 GHz radio; the network must be 2.4 GHz.
IPAddress STATIC_IP(192, 168, 1, 50);
IPAddress GATEWAY_IP(192, 168, 1, 1);
IPAddress SUBNET_MASK(255, 255, 255, 0);
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 20000;
const unsigned long WIFI_RETRY_INTERVAL_MS  = 30000;

// Optional: pin the association to one access point (see secrets.example.h).
// Useful when several APs broadcast the same network name and not all of them
// bridge onto the LAN your laptop is on - the board can then associate cleanly,
// accept its static IP, report itself connected, and still be unreachable.
// If the pinned AP does not answer, connectWiFi() falls back to any AP.
#ifdef WIFI_BSSID
const uint8_t PREFERRED_BSSID[6] = WIFI_BSSID;
#endif
// 0 = find that BSSID on whatever channel it is on. Keep it 0: routers change
// channel, and a pinned channel silently turns the pinned join into a timeout.
const int32_t PREFERRED_CHANNEL            = 0;
const unsigned long WIFI_PINNED_TIMEOUT_MS = 10000;

// ---- Pins ----
const int PIN_STEP    = 40;
const int PIN_DIR     = 38;
const int PIN_EN      = 10;  // active LOW
const int PIN_UART_TX = 17;  // -> 1k resistor -> PDN_UART
const int PIN_UART_RX = 18;  // -> PDN_UART directly
const int PIN_SERVO_A = 33;  // pusher servo A signal
const int PIN_SERVO_B = 34;  // pusher servo B signal
const int PIN_ENDSTOP = 35;  // INPUT_PULLUP, see ENDSTOP_ACTIVE_HIGH
const int PIN_LEVER   = 4;   // randomiser lever, INPUT_PULLUP, switch to GND

// ---- Homing ----
// Sign of stepper travel that moves the carriage TOWARD the endstop. Flip this
// if homing drives away from the switch. Everything else follows from it - see
// AXIS_SIGN and the note in the header.
const int HOME_DIRECTION = 1;
const long AXIS_SIGN = -HOME_DIRECTION; // logical +1 always points AWAY from home

// The endstop is a normally-CLOSED switch on an INPUT_PULLUP pin, so the resting
// (not-at-endstop) state pulls the pin LOW and the switch OPENS on contact,
// letting the pullup take it HIGH. Triggered is therefore HIGH, not LOW. This
// also gives the fail-safe behaviour the wiring was chosen for: a broken or
// disconnected endstop wire floats HIGH and reads as "triggered" rather than
// silently reading clear. Set false if the switch is ever rewired NO.
const bool ENDSTOP_ACTIVE_HIGH = true;

// Two-stage home: a fast approach to find the switch, a back-off, then a slow
// re-approach. The slow second touch is what makes the zero reference
// repeatable - a single fast approach lands a different distance past the
// trigger point every time, depending on how much momentum the carriage had.
const float HOME_SPEED_FAST   = 3000;
const float HOME_SPEED_SLOW   = 600;
const long  HOME_BACKOFF_STEPS = 600;
const long  HOME_OVERTRAVEL_STEPS = 800; // slack past railMaxSteps before giving up

// ---- Driver / motion config ----
#define R_SENSE 0.11f          // VERIFY against your specific module (0.11 for most stepstick boards)
#define DRIVER_ADDRESS 0b00
const int CURRENT_MAX_MA = 1400;  // hard ceiling for /current - the driver will happily cook itself above this
const int START_HOLD_CURRENT_MA = 300;
int currentMicrosteps = 8;
int currentRunCurrentMa = 1100;

// ---- Rail geometry ----
// Measured on the reference build: 200 steps ~= 0.5 cm at 8 microsteps ->
// 400 steps/cm. Measure yours (jog a known number of steps, measure the travel).
// steps/cm scales with microstepping, so changing microsteps invalidates this,
// railMaxSteps and every taught slot position. The dashboard warns about it.
const float STEPS_PER_CM = 400.0;
const float RAIL_LENGTH_CM = 30.0;
long railMaxSteps = (long)(STEPS_PER_CM * RAIL_LENGTH_CM);

float cruiseSpeed = 8000;        // steps/sec at full speed
float accelStepsPerSec2 = 20000; // steps/sec^2

// ---- Shelf slots ----
// EXAMPLE shelf: replace with your own. Slots are in physical left-to-right
// order and must line up 1:1 with SLOTS in app/projector.html. In this example
// slot 0 is the randomiser lever and 15/16 are empty cases, so they are never
// ejectable - but they still get entries, because the indices must match.
// When you change the count, update NUM_SLOTS and slotIsEjectable() too.
const int NUM_SLOTS = 17;

const char* SLOT_NAMES[NUM_SLOTS] = {
  "Randomizer: Roulette",
  "Donkey Kong Country Returns",
  "Disney Epic Mickey",
  "The Legend of Spyro: Dawn of the Dragon",
  "LEGO Star Wars: The Complete Saga",
  "LEGO Pirates of the Caribbean",
  "Guitar Hero: Greatest Hits",
  "EA Playground",
  "LEGO Indiana Jones 2",
  "Guitar Hero 5",
  "LEGO Batman 2: DC Super Heroes",
  "Example Game",
  "LEGO Star Wars III: The Clone Wars",
  "Cocoto Surprise",
  "World of Zoo",
  "(empty)",
  "(empty)"
};

// Slots that hold a pushable case. Keep in sync with SLOTS[i].kind in
// projector.html: in the example, index 0 is the lever and 15/16 are empty.
bool slotIsEjectable(int i) { return i > 0 && i < 15; }

long slotSteps[NUM_SLOTS];
bool slotSet[NUM_SLOTS];

// Which arm ejects each slot: 0 = pusher A (GPIO33), 1 = pusher B (GPIO34).
//
// This is stored per slot rather than derived from a boundary index. A clean
// left-half/right-half split is only true if the arms were mounted symmetrically
// and the shelf was never rearranged - neither is something the firmware should
// assume. The boundary still exists as a bulk-fill convenience (/api/split), but
// it is an action you apply, not the source of truth: after applying it you can
// still move any individual slot to the other arm.
const uint8_t PUSHER_A = 0;
const uint8_t PUSHER_B = 1;
uint8_t slotPusher[NUM_SLOTS];

// Boundary used the last time the bulk fill was applied. Reported in /api/status
// only so the dashboard can show the field pre-filled with something sensible.
const int DEFAULT_PUSHER_SPLIT_SLOT = 9;
int pusherSplitSlot = DEFAULT_PUSHER_SPLIT_SLOT;

// ---- Pusher timing ----
// "50% speed" is software-simulated: hobby servos don't take a speed command, so
// we step the angle gradually over time instead of jumping straight to the
// target. 100% speed sweeps the full 180deg range in SERVO_FULL_SWEEP_MS_100;
// 50% takes twice as long.
const unsigned long SERVO_FULL_SWEEP_MS_100 = 500;
const unsigned long SERVO_STEP_INTERVAL_MS  = 15;  // ~66 updates/sec, smooth without flooding the servo
const unsigned long SERVO_PUSH_WAIT_MS      = 1000;

// Rotation direction per pusher. On the reference build's MG90S servos a
// FALLING angle turns the horn clockwise. A sweeps rest->push downward, B
// upward, because the two arms are mirror images. If an arm pushes the wrong
// way on your build, swap that pusher's two constants - nothing else changes.
const int SERVO_A_REST_ANGLE = 180;  // GPIO33, clockwise on push
const int SERVO_A_PUSH_ANGLE = 0;
const int SERVO_B_REST_ANGLE = 0;    // GPIO34, counterclockwise on push
const int SERVO_B_PUSH_ANGLE = 180;

// Let the carriage stop ringing before the servo draws current. Also keeps the
// motor's holding current and the servo's stall current from peaking together,
// which is the pattern that trips brownouts on a shared supply.
const unsigned long SELECT_SETTLE_MS = 200;

const unsigned long INPUT_DEBOUNCE_MS = 30;

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------

HardwareSerial TMCSerial(1);
TMC2209Stepper driver(&TMCSerial, R_SENSE, DRIVER_ADDRESS);
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);
WebServer server(80);
Preferences prefs;

// Step generation runs from a hardware timer ISR instead of the main loop.
// The ESP32-S2 is single-core, so this - not core-pinning - is what actually
// decouples step timing from WiFi/WebServer work: an ISR preempts loop()
// no matter what it's doing.
hw_timer_t *motionTimer = NULL;
portMUX_TYPE stepperMux = portMUX_INITIALIZER_UNLOCKED;

bool driverEnabled = false;
bool homed = false;
bool wifiUp = false;

// ---------------------------------------------------------------------------
// WiFi black box
// ---------------------------------------------------------------------------
// WiFi problems tend to happen only when nothing is watching: on the shelf's own
// power supply, with no laptop attached - and the laptop is the only way to read
// Serial. So the board writes down what happened to its own WiFi on every boot,
// in NVS, and prints the PREVIOUS boot's record on '?'. Procedure: let it fail,
// plug in USB, send '?'.
//
// The reset reason is the single most useful field. BROWNOUT says the supply
// collapsed (and the board then reboots, re-homes, and tries again - a loop).
// POWERON with wifi=FAIL says the board ran fine and simply never associated,
// and then targetRssi separates "cannot hear the AP" from "hears it, cannot
// join".
//
// Declared by hand: both are defined far below but used by the '?' handler above
// them, and the Arduino preprocessor did not generate prototypes for these two.
const char* resetReasonName(int r);
const char* wifiStatusName(int s);

// Values, this boot:
int bbResetReason = 0;
uint32_t bbBoots = 0;
int bbWifiTries = 0;
int wifiTargetRssi = 0;   // from the failure scan: 0 = AP not seen, 1 = scan found nothing at all
// ...and as recorded by the boot before this one:
int prevResetReason = -1;
bool prevWifiOk = false;
int prevWifiTries = 0;
int prevWifiStatus = -1;
int prevWifiRssi = 0;
uint32_t prevWifiAtMs = 0;

// TX power cap. The radio's transmit bursts are the heaviest thing on the 3.3 V
// rail by a wide margin, and at full power (~20 dBm) they are the usual reason
// these mini boards fail to associate on a supply that measures a steady 5.0 V
// on a multimeter - a meter averages, the bursts are milliseconds. 11 dBm is
// plenty for a shelf in the same room as the router and takes a third or more
// off the peak current. Raise it if your router is far away.
const wifi_power_t WIFI_TX_POWER = WIFI_POWER_11dBm;
unsigned long lastWifiAttemptMs = 0;

Servo servoA;
Servo servoB;
int servoAAngle = SERVO_A_REST_ANGLE;
int servoBAngle = SERVO_B_REST_ANGLE;

ServoPusher pusherA(&servoA, &servoAAngle, SERVO_A_REST_ANGLE, SERVO_A_PUSH_ANGLE);
ServoPusher pusherB(&servoB, &servoBAngle, SERVO_B_REST_ANGLE, SERVO_B_PUSH_ANGLE);

LatchedInput leverInput(PIN_LEVER, true);

SelectPhase selPhase = SEL_IDLE;
int selSlot = -1;
// The ejection in progress was asked to leave the case out (/api/select?hold=1).
// Copied onto the pusher when it fires; see releasePushers().
bool selHold = false;
unsigned long selSettleStart = 0;

// ---- Demo sequence player: walks the taught slot table, for shakedown runs ----
const int MAX_SEQ = NUM_SLOTS;
int sequenceSlots[MAX_SEQ];
int sequenceLen = 0;
int sequenceIndex = 0;
bool sequencePlaying = false;
bool sequenceLooping = false;
bool sequencePushes = false;   // eject at each stop, or just visit it
bool sequenceDwelling = false;
unsigned long sequenceDwellStart = 0;
unsigned long sequenceDwellMs = 1000;

// ---------------------------------------------------------------------------
// Step ISR
// ---------------------------------------------------------------------------

// The rail is stepped from this 20 kHz tick. Stopping the alarm (see
// beginFlashCritical) freezes the motor, which is why every NVS write checks
// motorsIdle() first.
void IRAM_ATTR onMotionTimer() {
  portENTER_CRITICAL_ISR(&stepperMux);
  stepper.run();
  portEXIT_CRITICAL_ISR(&stepperMux);
}

// ---------------------------------------------------------------------------
// Flash-safe NVS writes
// ---------------------------------------------------------------------------
// onMotionTimer is IRAM_ATTR but stepper.run() is not - AccelStepper lives in
// flash. An NVS write disables the flash cache, so a timer interrupt landing in
// that window fetches from unmapped flash and panics the board. Teaching a slot
// position therefore has to stop the step timer for the duration of the write.
//
// That also means a save must never happen mid-travel: the alarm being off is
// indistinguishable from the motor being told to stop. Every caller checks
// railIsRunning() first.

void beginFlashCritical() { if (motionTimer) timerAlarmDisable(motionTimer); }
void endFlashCritical()   { if (motionTimer) timerAlarmEnable(motionTimer); }

// ---------------------------------------------------------------------------
// Logical axis
// ---------------------------------------------------------------------------
// 0 is the endstop, positive is away from it, always - regardless of which way
// the motor happens to be wired. See the header.

bool railIsRunning() {
  // Read without the mux on purpose. isRunning() touches two longs and a float,
  // all single-word aligned on a 32-bit core, so the read is atomic; taking the
  // critical section here instead would mean blocking the step ISR on every
  // pass of loop().
  return stepper.isRunning();
}

long railPosition() { return AXIS_SIGN * stepper.currentPosition(); }
long railTarget()   { return AXIS_SIGN * stepper.targetPosition(); }

void enableDriver(bool en) {
  driverEnabled = en;
  digitalWrite(PIN_EN, en ? LOW : HIGH); // LOW = enabled
}

void railMoveTo(long logicalPos) {
  logicalPos = constrain(logicalPos, 0L, railMaxSteps);
  enableDriver(true);
  portENTER_CRITICAL(&stepperMux);
  stepper.moveTo(AXIS_SIGN * logicalPos);
  portEXIT_CRITICAL(&stepperMux);
}

void railJog(long deltaLogical) {
  railMoveTo(railPosition() + deltaLogical);
}

void railStop() {
  portENTER_CRITICAL(&stepperMux);
  stepper.moveTo(stepper.currentPosition()); // distance 0 -> speed 0 on the next ISR pass
  portEXIT_CRITICAL(&stepperMux);
}

void railZeroHere() {
  portENTER_CRITICAL(&stepperMux);
  stepper.setCurrentPosition(0); // also clears any pending target - stationary callers only
  portEXIT_CRITICAL(&stepperMux);
}

bool endstopTriggered() {
  return digitalRead(PIN_ENDSTOP) == (ENDSTOP_ACTIVE_HIGH ? HIGH : LOW);
}

// Nothing stepping. The guard for every NVS write - see beginFlashCritical().
bool motorsIdle() { return !railIsRunning(); }

// ---------------------------------------------------------------------------
// Persistence
// ---------------------------------------------------------------------------

void persistSlots() {
  beginFlashCritical();
  prefs.putBytes("slots", slotSteps, sizeof(slotSteps));
  prefs.putBytes("slotpush", slotPusher, sizeof(slotPusher));
  uint32_t mask = 0;
  for (int i = 0; i < NUM_SLOTS; i++) if (slotSet[i]) mask |= (1UL << i);
  prefs.putUInt("slotmask", mask);
  endFlashCritical();
}

void persistRailMax() {
  beginFlashCritical();
  prefs.putLong("railmax", railMaxSteps);
  endFlashCritical();
}

void persistSplit() {
  beginFlashCritical();
  prefs.putInt("split", pusherSplitSlot);
  endFlashCritical();
}

// Speed and acceleration are changed from sliders, i.e. potentially dozens of
// times a second and often mid-move - exactly when an NVS write must not happen,
// since it stops the step timer. So the value applies immediately in RAM and the
// flash write is deferred until the rail is stationary (flushed from loop()).
bool motionSettingsDirty = false;

void persistMotionSettings() {
  beginFlashCritical();
  prefs.putFloat("speed", cruiseSpeed);
  prefs.putFloat("accel", accelStepsPerSec2);
  endFlashCritical();
  motionSettingsDirty = false;
}

void flushMotionSettings() {
  if (!motionSettingsDirty) return;
  if (!motorsIdle()) return; // stopping the timer mid-move would strand the carriage
  persistMotionSettings();
}

void loadSettings() {
  prefs.begin("irlhome", false);

  if (prefs.isKey("railmax")) railMaxSteps = prefs.getLong("railmax", railMaxSteps);
  if (prefs.isKey("split"))   pusherSplitSlot = prefs.getInt("split", DEFAULT_PUSHER_SPLIT_SLOT);
  if (prefs.isKey("speed"))   cruiseSpeed = prefs.getFloat("speed", cruiseSpeed);
  if (prefs.isKey("accel"))   accelStepsPerSec2 = prefs.getFloat("accel", accelStepsPerSec2);
  pusherSplitSlot = constrain(pusherSplitSlot, 0, NUM_SLOTS);
  cruiseSpeed = constrain(cruiseSpeed, 10.0f, 20000.0f);
  accelStepsPerSec2 = constrain(accelStepsPerSec2, 10.0f, 100000.0f);

  // Fresh NVS: seed the assignment from the default boundary so the shelf is
  // usable before anyone has been through it slot by slot.
  for (int i = 0; i < NUM_SLOTS; i++) {
    slotSteps[i] = 0;
    slotSet[i] = false;
    slotPusher[i] = (i < pusherSplitSlot) ? PUSHER_A : PUSHER_B;
  }

  if (prefs.getBytesLength("slots") == sizeof(slotSteps)) {
    prefs.getBytes("slots", slotSteps, sizeof(slotSteps));
    uint32_t mask = prefs.getUInt("slotmask", 0);
    for (int i = 0; i < NUM_SLOTS; i++) slotSet[i] = (mask >> i) & 1UL;
    Serial.println(">> SLOTS: loaded taught positions from NVS.");
  } else {
    Serial.println(">> SLOTS: nothing saved - every slot needs teaching.");
  }

  // Separate key and separate length check: an NVS written by an older build has
  // "slots" but no "slotpush", and that must fall back to the boundary seeding
  // above rather than leaving the array full of whatever getBytes did.
  if (prefs.getBytesLength("slotpush") == sizeof(slotPusher)) {
    prefs.getBytes("slotpush", slotPusher, sizeof(slotPusher));
    for (int i = 0; i < NUM_SLOTS; i++) {
      if (slotPusher[i] != PUSHER_A && slotPusher[i] != PUSHER_B) slotPusher[i] = PUSHER_A;
    }
    Serial.println(">> SLOTS: loaded per-slot pusher assignment from NVS.");
  }

  // A shrunken rail (someone re-ran SET RAIL END shorter) would otherwise leave
  // slots pointing past the soft limit, where railMoveTo silently clamps them
  // all onto the same spot.
  for (int i = 0; i < NUM_SLOTS; i++) {
    if (slotSet[i] && (slotSteps[i] < 0 || slotSteps[i] > railMaxSteps)) {
      Serial.printf(">> SLOTS: slot %d (%ld) is outside 0..%ld - dropping it.\n", i, slotSteps[i], railMaxSteps);
      slotSet[i] = false;
    }
  }
}

// Calibrate the far end of the rail against wherever the cart is right now -
// the mirror image of homing for the near end.
bool setEndHere() {
  if (!motorsIdle()) return false;
  long pos = railPosition();
  if (pos <= 0) return false; // nonsensical as an end if it's at or before home
  railMaxSteps = pos;
  persistRailMax();
  return true;
}

bool teachSlot(int slot, long steps) {
  if (slot < 0 || slot >= NUM_SLOTS) return false;
  if (!motorsIdle()) return false; // NVS write stops the step timer - see beginFlashCritical
  slotSteps[slot] = constrain(steps, 0L, railMaxSteps);
  slotSet[slot] = true;
  persistSlots();
  Serial.printf(">> SLOTS: slot %d (%s) taught at %ld\n", slot, SLOT_NAMES[slot], slotSteps[slot]);
  return true;
}

bool clearSlot(int slot) {
  if (slot < 0 || slot >= NUM_SLOTS) return false;
  if (!motorsIdle()) return false;
  slotSet[slot] = false;
  slotSteps[slot] = 0;
  persistSlots();
  return true;
}

void resetSlots() {
  if (!motorsIdle()) return;
  for (int i = 0; i < NUM_SLOTS; i++) { slotSteps[i] = 0; slotSet[i] = false; }
  persistSlots();
}

// ---------------------------------------------------------------------------
// Homing
// ---------------------------------------------------------------------------
// Blocking. Nothing else can meaningfully happen while the carriage is hunting
// for its zero reference, and the web server not answering for a few seconds is
// preferable to a second /api/home arriving halfway through the first one. The
// caller sends its HTTP response before invoking this.

// Drive until the endstop changes to `wantTriggered`, or we run out of rail.
// Returns true if the switch reached the wanted state.
bool homeSeek(float speed, long maxSteps, bool wantTriggered) {
  long dir = wantTriggered ? HOME_DIRECTION : -HOME_DIRECTION;

  portENTER_CRITICAL(&stepperMux);
  stepper.setMaxSpeed(speed);
  stepper.setAcceleration(accelStepsPerSec2);
  stepper.move(dir * maxSteps);
  portEXIT_CRITICAL(&stepperMux);

  while (railIsRunning()) {
    if (endstopTriggered() == wantTriggered) {
      portENTER_CRITICAL(&stepperMux);
      stepper.moveTo(stepper.currentPosition()); // cancel remaining travel immediately
      portEXIT_CRITICAL(&stepperMux);
      return true;
    }
    delay(1); // the timer ISR keeps stepping in the background; just watch the switch
  }
  return endstopTriggered() == wantTriggered;
}

void homeAxis() {
  Serial.println(">> HOME: starting");
  homed = false;
  selPhase = SEL_IDLE;
  selSlot = -1;
  sequencePlaying = false;
  enableDriver(true);

  // Travel budget: the whole rail plus slack, so a missing switch gives up
  // after one rail length instead of grinding against the end stop.
  const long budget = railMaxSteps + HOME_OVERTRAVEL_STEPS;

  // Already sitting on the switch: back off first, or the fast seek has nothing
  // to find and reports success without ever moving.
  if (endstopTriggered()) {
    Serial.println(">> HOME: already triggered, backing off");
    if (!homeSeek(HOME_SPEED_SLOW, HOME_BACKOFF_STEPS * 2, false)) {
      Serial.println(">> HOME: FAILED - endstop still reads triggered after back-off (wiring?)");
      railStop();
      portENTER_CRITICAL(&stepperMux);
      stepper.setMaxSpeed(cruiseSpeed);
      portEXIT_CRITICAL(&stepperMux);
      return;
    }
  }

  if (!homeSeek(HOME_SPEED_FAST, budget, true)) {
    Serial.println(">> HOME: FAILED - switch not found within one rail length");
    railStop();
    railZeroHere(); // leave a sane reference rather than a huge stale count
    portENTER_CRITICAL(&stepperMux);
    stepper.setMaxSpeed(cruiseSpeed);
    portEXIT_CRITICAL(&stepperMux);
    return;
  }

  if (!homeSeek(HOME_SPEED_SLOW, HOME_BACKOFF_STEPS * 3, false)) {
    Serial.println(">> HOME: FAILED - could not clear the switch on back-off");
    railStop();
    railZeroHere();
    portENTER_CRITICAL(&stepperMux);
    stepper.setMaxSpeed(cruiseSpeed);
    portEXIT_CRITICAL(&stepperMux);
    return;
  }

  // Slow second touch - this is the one that defines zero.
  bool ok = homeSeek(HOME_SPEED_SLOW, HOME_BACKOFF_STEPS * 3, true);

  portENTER_CRITICAL(&stepperMux);
  if (ok) {
    stepper.setCurrentPosition(0);
    homed = true;
  }
  stepper.setMaxSpeed(cruiseSpeed); // restore normal cruise speed
  portEXIT_CRITICAL(&stepperMux);

  Serial.println(homed ? ">> HOME: OK, zero set" : ">> HOME: FAILED on slow re-approach");
}

// ---------------------------------------------------------------------------
// Pushers
// ---------------------------------------------------------------------------

ServoPusher& pusherForSlot(int slot) {
  if (slot < 0 || slot >= NUM_SLOTS) return pusherA;
  return (slotPusher[slot] == PUSHER_B) ? pusherB : pusherA;
}

const char* pusherNameForSlot(int slot) {
  if (slot < 0 || slot >= NUM_SLOTS) return "?";
  return (slotPusher[slot] == PUSHER_B) ? "B" : "A";
}

// Changing the arm for a slot invalidates that slot's taught position: the two
// arms sit at different points on the carriage, so the position that lined A up
// with the case puts B somewhere else entirely. Rather than silently ejecting
// the wrong case, the position is dropped and the slot goes back to untaught.
bool setSlotPusher(int slot, uint8_t which) {
  if (slot < 0 || slot >= NUM_SLOTS) return false;
  if (railIsRunning()) return false; // NVS write stops the step timer
  if (slotPusher[slot] != which && slotSet[slot]) {
    slotSet[slot] = false;
    slotSteps[slot] = 0;
    Serial.printf(">> SLOTS: slot %d reassigned to pusher %s - position cleared, re-teach it\n",
                  slot, which == PUSHER_B ? "B" : "A");
  }
  slotPusher[slot] = which;
  persistSlots();
  return true;
}

// Bulk fill: everything below `boundary` to A, everything at or above it to B.
// A starting point for a fresh shelf, not a constraint - individual slots can be
// moved afterwards.
void applyPusherSplit(int boundary) {
  pusherSplitSlot = constrain(boundary, 0, NUM_SLOTS);
  for (int i = 0; i < NUM_SLOTS; i++) {
    uint8_t want = (i < pusherSplitSlot) ? PUSHER_A : PUSHER_B;
    if (slotPusher[i] != want && slotSet[i]) { slotSet[i] = false; slotSteps[i] = 0; }
    slotPusher[i] = want;
  }
  persistSlots();
  persistSplit();
}

bool pushersIdle() {
  return pusherA.phase == PUSH_IDLE && pusherB.phase == PUSH_IDLE;
}

// True while an arm is parked out in the shelf. Anything that would move the
// carriage sideways has to refuse while this holds - the arm is between two
// cases, and the rail does not care what is in its way.
bool pusherHeld() {
  return (pusherA.hold && pusherA.phase != PUSH_IDLE) ||
         (pusherB.hold && pusherB.phase != PUSH_IDLE);
}

// Let go of any held case. A pusher sitting in PUSH_WAIT with an expired timer
// retracts on the very next service pass, so this is all a retract has to do.
void releasePushers() {
  pusherA.hold = false;
  pusherB.hold = false;
  selHold = false;
}

void startPush(ServoPusher &p) {
  if (p.phase != PUSH_IDLE) return; // already mid-cycle, ignore a re-trigger
  p.currentAngleF = *p.angle;
  p.phase = PUSH_DOWN;
  p.lastStepMs = millis();
}

void servicePusher(ServoPusher &p) {
  unsigned long now = millis();

  if (p.phase == PUSH_IDLE) {
    // Re-assert the resting angle periodically. A servo written once per state
    // change goes limp when that single write gets dropped, and a limp pusher
    // sags into the case behind it. Cheap enough to just keep saying it.
    if (now - p.lastStepMs >= 250) {
      p.lastStepMs = now;
      p.servo->write(*p.angle);
    }
    return;
  }

  if (p.phase == PUSH_WAIT) {
    p.servo->write(*p.angle); // hold the extended position against the case
    // A held push still serves out the minimum wait first, so a retract that
    // arrives the instant the arm lands does not snap it straight back.
    if (!p.hold && now - p.waitStart >= SERVO_PUSH_WAIT_MS) {
      p.phase = PUSH_UP;
      p.lastStepMs = now;
    }
    return;
  }

  if (now - p.lastStepMs < SERVO_STEP_INTERVAL_MS) return;
  p.lastStepMs = now;

  // 50% speed -> full sweep takes 2x SERVO_FULL_SWEEP_MS_100
  float degPerStep = 180.0 * SERVO_STEP_INTERVAL_MS / (float)(SERVO_FULL_SWEEP_MS_100 * 2);

  // Travel direction falls out of this pusher's own rest/push angles rather than
  // being hardcoded, so A (GPIO33) sweeps clockwise and B (GPIO34) counterclockwise
  // through the same routine.
  int target = (p.phase == PUSH_DOWN) ? p.pushAngle : p.restAngle;
  bool rising = (float)target > p.currentAngleF;
  p.currentAngleF += rising ? degPerStep : -degPerStep;

  bool arrived = rising ? (p.currentAngleF >= (float)target)
                        : (p.currentAngleF <= (float)target);
  if (arrived) {
    p.currentAngleF = (float)target;
    p.servo->write(target);
    *p.angle = target;
    if (p.phase == PUSH_DOWN) {
      p.phase = PUSH_WAIT;
      p.waitStart = now;
    } else {
      p.phase = PUSH_IDLE;
      p.hold = false;
    }
    return;
  }
  p.servo->write((int)p.currentAngleF);
  *p.angle = (int)p.currentAngleF;
}

void servicePushers() {
  servicePusher(pusherA);
  servicePusher(pusherB);
}

// Which taught slot the carriage is closest to right now. Used by /api/testpush,
// which has no slot argument - it fires "the pusher that serves wherever I am".
int nearestTaughtSlot() {
  int best = -1;
  long bestDist = 0;
  long pos = railPosition();
  for (int i = 0; i < NUM_SLOTS; i++) {
    if (!slotSet[i]) continue;
    long d = labs(slotSteps[i] - pos);
    if (best == -1 || d < bestDist) { best = i; bestDist = d; }
  }
  return best;
}

// ---------------------------------------------------------------------------
// Select: drive to a slot, then eject it
// ---------------------------------------------------------------------------

// reason is filled in on failure so the HTTP layer can say something useful.
bool startSelect(int slot, String &reason, bool hold) {
  if (slot < 0 || slot >= NUM_SLOTS) { reason = "bad slot"; return false; }
  if (!homed)                        { reason = "not homed"; return false; }
  if (!slotSet[slot])                { reason = "slot not taught"; return false; }
  if (!slotIsEjectable(slot))        { reason = "slot holds no case"; return false; }
  if (selPhase != SEL_IDLE)          { reason = "busy"; return false; }
  if (!pushersIdle())                { reason = "pusher mid-cycle"; return false; }

  sequencePlaying = false; // an explicit pick outranks a demo run
  selSlot = slot;
  selHold = hold;
  railMoveTo(slotSteps[slot]);
  selPhase = SEL_TRAVEL;
  Serial.printf(">> SELECT: slot %d (%s) -> %ld, pusher %s%s\n",
                slot, SLOT_NAMES[slot], slotSteps[slot], pusherNameForSlot(slot),
                hold ? " (hold out)" : "");
  return true;
}

void abortSelect() {
  releasePushers(); // STOP must never leave an arm parked out in the shelf
  selPhase = SEL_IDLE;
  selSlot = -1;
  railStop();
}

void serviceSelect() {
  if (selPhase == SEL_IDLE) return;

  if (selPhase == SEL_TRAVEL) {
    if (railIsRunning()) return;
    selPhase = SEL_SETTLE;
    selSettleStart = millis();
    return;
  }

  if (selPhase == SEL_SETTLE) {
    if (millis() - selSettleStart < SELECT_SETTLE_MS) return;
    pusherForSlot(selSlot).hold = selHold;
    startPush(pusherForSlot(selSlot));
    selPhase = SEL_PUSH;
    return;
  }

  // SEL_PUSH - the carriage stays where it is once the case is out. Driving
  // back to a park position here would just fight the next navigation, and
  // there is nothing for it to get out of the way of.
  if (pusherForSlot(selSlot).phase != PUSH_IDLE) return;
  Serial.printf(">> SELECT: slot %d done\n", selSlot);
  selPhase = SEL_IDLE;
  selSlot = -1;
}

// ---------------------------------------------------------------------------
// Latched input (lever)
// ---------------------------------------------------------------------------

void serviceLatchedInput(LatchedInput &in) {
  bool reading = digitalRead(in.pin) == (in.activeLow ? LOW : HIGH);
  unsigned long now = millis();

  if (reading != in.raw) {
    in.raw = reading;
    in.lastChangeMs = now;
    return; // still bouncing - decide on a later pass, not this one
  }

  if (now - in.lastChangeMs < INPUT_DEBOUNCE_MS) return;
  if (reading == in.settled) return;

  in.settled = reading;
  if (reading) {           // active edge
    in.latched = true;
    in.lastEventMs = now;
  }
}

// Collect and clear. The latch is what survives the gap between the app's
// 200 ms polls.
bool takeLatch(LatchedInput &in) {
  bool v = in.latched;
  in.latched = false;
  return v;
}

// ---------------------------------------------------------------------------
// Demo sequence
// ---------------------------------------------------------------------------

void seqAdd(int slot) {
  if (sequenceLen >= MAX_SEQ) return;
  if (slot < 0 || slot >= NUM_SLOTS || !slotSet[slot]) return;
  sequenceSlots[sequenceLen++] = slot;
}

void seqClear() {
  sequenceLen = 0;
  sequenceIndex = 0;
  sequencePlaying = false;
  sequenceDwelling = false;
}

void seqPlay(unsigned long dwellMs, bool loopFlag, bool pushFlag) {
  if (sequenceLen == 0 || !homed) return;
  sequenceDwellMs = dwellMs;
  sequenceLooping = loopFlag;
  sequencePushes = pushFlag;
  sequenceIndex = 0;
  sequenceDwelling = false;
  sequencePlaying = true;
  railMoveTo(slotSteps[sequenceSlots[0]]);
}

void seqStop() {
  sequencePlaying = false;
  sequenceDwelling = false;
}

void serviceSequence() {
  if (!sequencePlaying) return;
  if (selPhase != SEL_IDLE) return; // an ejection from this sequence is still finishing
  if (railIsRunning()) return;
  if (!pushersIdle()) return;

  if (!sequenceDwelling) {
    sequenceDwelling = true;
    sequenceDwellStart = millis();
    if (sequencePushes && slotIsEjectable(sequenceSlots[sequenceIndex])) {
      startPush(pusherForSlot(sequenceSlots[sequenceIndex]));
    }
    return;
  }
  if (millis() - sequenceDwellStart < sequenceDwellMs) return;

  sequenceDwelling = false;
  sequenceIndex++;
  if (sequenceIndex >= sequenceLen) {
    if (sequenceLooping) {
      sequenceIndex = 0;
    } else {
      sequencePlaying = false;
      return;
    }
  }
  railMoveTo(slotSteps[sequenceSlots[sequenceIndex]]);
}

// ---------------------------------------------------------------------------
// Driver tuning
// ---------------------------------------------------------------------------

// Always re-derive the hold multiplier. The single-argument rms_current()
// overload resets holding current to 50% of run current, so changing run
// current from the UI used to silently double what the motor burns at standstill.
void applyRunCurrent(int ma) {
  currentRunCurrentMa = constrain(ma, 100, CURRENT_MAX_MA);
  float holdMultiplier = (float)START_HOLD_CURRENT_MA / (float)currentRunCurrentMa;
  if (holdMultiplier > 1.0) holdMultiplier = 1.0;
  driver.rms_current(currentRunCurrentMa, holdMultiplier);
}

bool applyMicrosteps(int n) {
  switch (n) {
    case 1: case 2: case 4: case 8: case 16: case 32: case 64: case 128: case 256:
      currentMicrosteps = n;
      driver.microsteps(n);
      return true;
    default:
      return false;
  }
}

// ---------------------------------------------------------------------------
// Serial control
// ---------------------------------------------------------------------------

void printSlotTable() {
  for (int i = 0; i < NUM_SLOTS; i++) {
    Serial.printf("  %2d  %-44s  %-8s  pusher %s%s\n",
                  i, SLOT_NAMES[i],
                  slotSet[i] ? String(slotSteps[i]).c_str() : "-",
                  pusherNameForSlot(i),
                  slotIsEjectable(i) ? "" : "   (not ejectable)");
  }
}

void handleSerialLine(String line) {
  line.trim();
  if (line.length() == 0) return;
  char cmd = line.charAt(0);
  String rest = line.substring(1);
  rest.trim();
  String reason;

  switch (cmd) {
    case 'j': railJog(rest.toInt()); break;
    case 'm': railMoveTo(rest.toInt()); break;
    case 'z': railZeroHere(); homed = true; break;
    case 'h': homeAxis(); break;
    case 'x': Serial.println(setEndHere() ? "rail end set" : "refused (moving, or at/before 0)"); break;
    case 'g': {
      int idx = rest.toInt();
      if (idx < 0 || idx >= NUM_SLOTS) { Serial.println("bad slot"); break; }
      if (!homed) { Serial.println("not homed"); break; }
      if (!slotSet[idx]) { Serial.println("slot not taught"); break; }
      railMoveTo(slotSteps[idx]);
      break;
    }
    case 'G':
      if (!startSelect(rest.toInt(), reason, false)) Serial.println("select refused: " + reason);
      break;
    case 'k': Serial.println(teachSlot(rest.toInt(), railPosition()) ? "taught" : "refused"); break;
    case 'K': printSlotTable(); break;
    case 'A': Serial.println(setSlotPusher(rest.toInt(), PUSHER_A) ? "slot -> pusher A" : "refused"); break;
    case 'B': Serial.println(setSlotPusher(rest.toInt(), PUSHER_B) ? "slot -> pusher B" : "refused"); break;
    case 's': cruiseSpeed = constrain(rest.toFloat(), 10, 20000); stepper.setMaxSpeed(cruiseSpeed); break;
    case 'a': accelStepsPerSec2 = constrain(rest.toFloat(), 10, 100000); stepper.setAcceleration(accelStepsPerSec2); break;
    case 'e': enableDriver(rest.toInt() != 0); break;
    case 'c': applyRunCurrent(rest.toInt()); break;
    case 'u': if (!applyMicrosteps(rest.toInt())) Serial.println("bad microstep value"); break;
    case 't': driver.en_spreadCycle(rest.toInt() == 0); break;
    case 'p': servoAAngle = constrain(rest.toInt(), 0, 180); servoA.write(servoAAngle); break;
    case 'q': servoBAngle = constrain(rest.toInt(), 0, 180); servoB.write(servoBAngle); break;
    case 'P': startPush(pusherA); break;
    case 'Q': startPush(pusherB); break;
    case 'T': {
      // Read-only UART probe: test_connection() reads a register, changes nothing.
      // FAIL means the PDN_UART wiring (or the 1 kOhm resistor) is wrong, or VM is
      // not powered - the driver does not answer on UART without motor power.
      uint8_t r = driver.test_connection();
      Serial.printf("TMC2209 uart=%s\n", r == 0 ? "OK" : "FAIL (check PDN_UART wiring and motor power)");
      break;
    }
    case '?':
      Serial.printf("pos=%ld target=%ld running=%d homed=%d endstop=%d enabled=%d railMax=%ld\n",
                    railPosition(), railTarget(), railIsRunning(), homed,
                    endstopTriggered(), driverEnabled, railMaxSteps);
      Serial.printf("select=%d slot=%d split=%d servoA=%d servoB=%d lever=%d ip=%s\n",
                    (int)selPhase, selSlot, pusherSplitSlot, servoAAngle, servoBAngle,
                    leverInput.settled,
                    wifiUp ? WiFi.localIP().toString().c_str() : "offline");
      if (wifiUp) {
        // Which access point, not just which IP - see PREFERRED_BSSID. Queryable
        // here rather than only at boot because this board's USB is native: a
        // reset drops the port, so the boot banner is hard to capture.
        Serial.printf("bssid=%s ch=%d rssi=%d mac=%s mask=%s gw=%s\n",
                      WiFi.BSSIDstr().c_str(), WiFi.channel(), WiFi.RSSI(),
                      WiFi.macAddress().c_str(),
                      WiFi.subnetMask().toString().c_str(),
                      WiFi.gatewayIP().toString().c_str());
      }
      // The WiFi black box - see its declaration. "prev" is the boot before this
      // one, which is the one that matters when this boot only exists because a
      // laptop was plugged in to ask.
      Serial.printf("boot#%u reset=%s wifiTries=%d txPower=%d\n",
                    (unsigned)bbBoots, resetReasonName(bbResetReason), bbWifiTries,
                    (int)WiFi.getTxPower());
      Serial.printf("prevBoot: reset=%s wifi=%s tries=%d status=%s rssi=%d at=%us\n",
                    resetReasonName(prevResetReason), prevWifiOk ? "OK" : "FAIL",
                    prevWifiTries, wifiStatusName(prevWifiStatus), prevWifiRssi,
                    (unsigned)(prevWifiAtMs / 1000));
      break;
    default:
      Serial.println("unknown command");
  }
}

void pollSerial() {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      handleSerialLine(buf);
      buf = "";
    } else if (c != '\r') {
      buf += c;
    }
  }
}

// ---------------------------------------------------------------------------
// HTTP
// ---------------------------------------------------------------------------

void addCORS() { server.sendHeader("Access-Control-Allow-Origin", "*"); }

void sendPlain(int code, String msg) {
  addCORS();
  server.send(code, "text/plain", msg);
}

const char* pushPhaseName(PushPhase p) {
  switch (p) {
    case PUSH_DOWN: return "down";
    case PUSH_WAIT: return "hold";
    case PUSH_UP:   return "up";
    default:        return "idle";
  }
}

const char* selPhaseName(SelectPhase p) {
  switch (p) {
    case SEL_TRAVEL: return "travel";
    case SEL_SETTLE: return "settle";
    case SEL_PUSH:   return "push";
    default:         return "idle";
  }
}

void handleStatus() {
  addCORS();
  String body = "{";
  body += "\"pos\":" + String(railPosition()) + ",";
  body += "\"target\":" + String(railTarget()) + ",";
  body += "\"moving\":" + String(railIsRunning() ? "true" : "false") + ",";
  body += "\"railMaxSteps\":" + String(railMaxSteps) + ",";
  body += "\"maxSpeed\":" + String(cruiseSpeed, 0) + ",";
  body += "\"accel\":" + String(accelStepsPerSec2, 0) + ",";
  body += "\"current\":" + String(currentRunCurrentMa) + ",";
  body += "\"microsteps\":" + String(currentMicrosteps) + ",";
  body += "\"enabled\":" + String(driverEnabled ? "true" : "false") + ",";
  body += "\"homed\":" + String(homed ? "true" : "false") + ",";
  body += "\"endstop\":" + String(endstopTriggered() ? "true" : "false") + ",";
  body += "\"selectPhase\":\"" + String(selPhaseName(selPhase)) + "\",";
  body += "\"selectSlot\":" + String(selSlot) + ",";
  body += "\"held\":" + String(pusherHeld() ? "true" : "false") + ",";
  body += "\"splitSlot\":" + String(pusherSplitSlot) + ",";
  body += "\"servoA\":" + String(servoAAngle) + ",";
  body += "\"servoB\":" + String(servoBAngle) + ",";
  body += "\"pusherA\":\"" + String(pushPhaseName(pusherA.phase)) + "\",";
  body += "\"pusherB\":\"" + String(pushPhaseName(pusherB.phase)) + "\",";
  body += "\"lever\":" + String(leverInput.settled ? "true" : "false");
  body += "}";
  server.send(200, "application/json", body);
}

void handleSlotsList() {
  addCORS();
  String body = "[";
  for (int i = 0; i < NUM_SLOTS; i++) {
    if (i) body += ",";
    body += "{\"i\":" + String(i);
    body += ",\"name\":\"" + String(SLOT_NAMES[i]) + "\"";
    body += ",\"step\":" + String(slotSteps[i]);
    body += ",\"set\":" + String(slotSet[i] ? "true" : "false");
    body += ",\"ejectable\":" + String(slotIsEjectable(i) ? "true" : "false");
    body += ",\"pusher\":\"" + String(pusherNameForSlot(i)) + "\"}";
  }
  body += "]";
  server.send(200, "application/json", body);
}

void handleSelect() {
  if (!server.hasArg("game")) { sendPlain(400, "missing game"); return; }
  String reason;
  // hold=1: push the case out and LEAVE it out until /api/retract. selPhase
  // stays SEL_PUSH for the whole time it is out, which is what keeps /api/goto
  // answering "busy" - the carriage cannot be navigated away from under the arm.
  bool hold = server.hasArg("hold") && server.arg("hold").toInt() != 0;
  if (!startSelect(server.arg("game").toInt(), reason, hold)) { sendPlain(409, reason); return; }
  sendPlain(200, hold ? "OK selecting (hold)" : "OK selecting");
}

// Put a held case back. Always answers 200: "nothing was out" is not an error
// for a client whose only goal is that nothing is out.
void handleRetract() {
  bool wasHeld = pusherHeld() || selHold;
  // Asked to put it back before it ever went out: call the ejection off rather
  // than letting the arm pop out and straight back in.
  if (selPhase == SEL_TRAVEL || selPhase == SEL_SETTLE) abortSelect();
  releasePushers();
  sendPlain(200, wasHeld ? "OK retracting" : "OK nothing held");
}

void handleGoto() {
  if (!server.hasArg("game")) { sendPlain(400, "missing game"); return; }
  int idx = server.arg("game").toInt();
  if (idx < 0 || idx >= NUM_SLOTS) { sendPlain(400, "bad slot"); return; }
  if (!homed) { sendPlain(409, "not homed"); return; }
  if (!slotSet[idx]) { sendPlain(409, "slot not taught"); return; }
  // A goto is a navigation hint, not a command - never let it interrupt an
  // ejection that is already underway, or the pusher fires mid-travel.
  if (selPhase != SEL_IDLE) { sendPlain(409, "busy"); return; }
  sequencePlaying = false;
  railMoveTo(slotSteps[idx]);
  sendPlain(200, "OK");
}

void handleHome() {
  if (pusherHeld()) { sendPlain(409, "pusher held out"); return; }
  // Answer before blocking: homing takes seconds and the server is single
  // threaded, so the client would otherwise sit on a dead socket the whole time.
  sendPlain(200, "OK homing");
  homeAxis();
}

void handleStop() {
  abortSelect();
  seqStop();
  sendPlain(200, "OK stopped");
}

void handleTestPush() {
  if (railIsRunning()) { sendPlain(409, "moving"); return; }
  if (!pushersIdle())  { sendPlain(409, "pusher busy"); return; }

  String which = server.hasArg("which") ? server.arg("which") : "";
  if (which == "a" || which == "A") { startPush(pusherA); sendPlain(200, "OK push A"); return; }
  if (which == "b" || which == "B") { startPush(pusherB); sendPlain(200, "OK push B"); return; }

  // No argument: fire whichever half serves wherever the carriage is standing.
  int slot = nearestTaughtSlot();
  if (slot == -1) { sendPlain(409, "no taught slots - cannot tell which pusher"); return; }
  startPush(pusherForSlot(slot));
  sendPlain(200, "OK push " + String(pusherNameForSlot(slot)) + " (slot " + String(slot) + ")");
}

void handleSetSlot() {
  if (!server.hasArg("game")) { sendPlain(400, "missing game"); return; }
  int idx = server.arg("game").toInt();
  if (idx < 0 || idx >= NUM_SLOTS) { sendPlain(400, "bad slot"); return; }

  // Assign the arm first if one was named. Doing it the other way round would
  // trip setSlotPusher's "reassignment clears the position" rule and throw away
  // the position we just taught.
  if (server.hasArg("pusher")) {
    String p = server.arg("pusher");
    uint8_t which = (p == "b" || p == "B" || p == "1") ? PUSHER_B : PUSHER_A;
    if (!setSlotPusher(idx, which)) { sendPlain(409, "refused (still moving)"); return; }
  }

  long steps = server.hasArg("steps") ? server.arg("steps").toInt() : railPosition();
  if (!teachSlot(idx, steps)) { sendPlain(409, "refused (bad slot, or still moving)"); return; }
  sendPlain(200, String(slotSteps[idx]) + " " + pusherNameForSlot(idx));
}

void handleSlotPusher() {
  if (!server.hasArg("game")) { sendPlain(400, "missing game"); return; }
  if (!server.hasArg("pusher")) { sendPlain(400, "missing pusher"); return; }
  int idx = server.arg("game").toInt();
  String p = server.arg("pusher");
  uint8_t which = (p == "b" || p == "B" || p == "1") ? PUSHER_B : PUSHER_A;
  if (!setSlotPusher(idx, which)) { sendPlain(409, "refused (bad slot, or still moving)"); return; }
  sendPlain(200, "OK slot " + String(idx) + " -> pusher " + pusherNameForSlot(idx));
}

void handleClearSlot() {
  if (!server.hasArg("game")) { sendPlain(400, "missing game"); return; }
  sendPlain(clearSlot(server.arg("game").toInt()) ? 200 : 409, "OK");
}

void handleResetSlots() {
  resetSlots();
  sendPlain(200, "OK cleared");
}

void handleSplit() {
  if (!server.hasArg("slot")) { sendPlain(400, "missing slot"); return; }
  if (railIsRunning()) { sendPlain(409, "moving"); return; } // the NVS write stops the step timer
  applyPusherSplit(server.arg("slot").toInt());
  sendPlain(200, "OK filled: slots 0-" + String(pusherSplitSlot - 1) + " -> A, " +
                 String(pusherSplitSlot) + "-" + String(NUM_SLOTS - 1) + " -> B");
}

void handleSpinStatus() {
  addCORS();
  server.send(200, "application/json",
              String("{\"pressed\":") + (takeLatch(leverInput) ? "true" : "false") + "}");
}

void handleJog() {
  if (!server.hasArg("steps")) { sendPlain(400, "missing steps"); return; }
  if (!homed) { sendPlain(409, "not homed"); return; }
  if (pusherHeld()) { sendPlain(409, "pusher held out"); return; }
  railJog(server.arg("steps").toInt());
  sendPlain(200, "OK jogging");
}

void handleMoveTo() {
  if (!server.hasArg("pos")) { sendPlain(400, "missing pos"); return; }
  if (!homed) { sendPlain(409, "not homed"); return; }
  if (pusherHeld()) { sendPlain(409, "pusher held out"); return; }
  railMoveTo(server.arg("pos").toInt());
  sendPlain(200, "OK moving");
}

void handleZero() {
  if (!motorsIdle()) { sendPlain(409, "moving"); return; }
  railZeroHere();
  homed = true; // a manual zero is a deliberate reference, same standing as a homed one
  sendPlain(200, "OK zeroed");
}

void handleSetEnd() {
  if (!setEndHere()) { sendPlain(409, "refused (moving, or at/before 0)"); return; }
  sendPlain(200, "OK railMaxSteps=" + String(railMaxSteps));
}

void handleServoA() {
  if (!server.hasArg("angle")) { sendPlain(400, "missing angle"); return; }
  servoAAngle = constrain(server.arg("angle").toInt(), 0, 180);
  servoA.write(servoAAngle);
  sendPlain(200, "OK servoA=" + String(servoAAngle));
}

void handleServoB() {
  if (!server.hasArg("angle")) { sendPlain(400, "missing angle"); return; }
  servoBAngle = constrain(server.arg("angle").toInt(), 0, 180);
  servoB.write(servoBAngle);
  sendPlain(200, "OK servoB=" + String(servoBAngle));
}

void handlePushA() { startPush(pusherA); sendPlain(200, "OK pushing A"); }
void handlePushB() { startPush(pusherB); sendPlain(200, "OK pushing B"); }

void handleCurrent() {
  if (!server.hasArg("ma")) { sendPlain(400, "missing ma"); return; }
  applyRunCurrent(server.arg("ma").toInt());
  sendPlain(200, "OK current=" + String(currentRunCurrentMa) + "mA");
}

void handleMicrosteps() {
  if (!server.hasArg("n")) { sendPlain(400, "missing n"); return; }
  if (!applyMicrosteps(server.arg("n").toInt())) { sendPlain(400, "bad microstep value"); return; }
  sendPlain(200, "OK microsteps=" + String(currentMicrosteps));
}

void handleStealth() {
  bool stealth = server.hasArg("on") && server.arg("on") == "1";
  driver.en_spreadCycle(!stealth);
  sendPlain(200, stealth ? "stealthchop" : "spreadcycle");
}

void handleSpeed() {
  if (server.hasArg("stepspersec")) {
    cruiseSpeed = constrain(server.arg("stepspersec").toFloat(), 10, 20000);
    stepper.setMaxSpeed(cruiseSpeed);
    motionSettingsDirty = true; // written to NVS once the rail is stationary
  }
  sendPlain(200, "OK maxSpeed=" + String(cruiseSpeed, 0));
}

void handleAccel() {
  if (server.hasArg("stepspersec2")) {
    accelStepsPerSec2 = constrain(server.arg("stepspersec2").toFloat(), 10, 100000);
    stepper.setAcceleration(accelStepsPerSec2);
    motionSettingsDirty = true;
  }
  sendPlain(200, "OK accel=" + String(accelStepsPerSec2, 0));
}

void handleEnable() {
  bool en = server.hasArg("on") && server.arg("on") == "1";
  enableDriver(en);
  // Letting go of the motor means the tracked position stops meaning anything
  // the moment someone slides the carriage by hand.
  if (!en) homed = false;
  sendPlain(200, en ? "enabled" : "disabled (position reference dropped)");
}

void handleSeqAdd() {
  if (!server.hasArg("slot")) { sendPlain(400, "missing slot"); return; }
  seqAdd(server.arg("slot").toInt());
  sendPlain(200, "OK added");
}

void handleSeqClear() { seqClear(); sendPlain(200, "OK cleared"); }
void handleSeqStop()  { seqStop();  sendPlain(200, "OK stopped"); }

void handleSeqPlay() {
  unsigned long dwell = server.hasArg("dwell") ? server.arg("dwell").toInt() : 1000;
  bool loopFlag = server.hasArg("loop") && server.arg("loop") == "1";
  bool pushFlag = server.hasArg("push") && server.arg("push") == "1";
  seqPlay(dwell, loopFlag, pushFlag);
  sendPlain(200, homed ? "OK playing" : "refused (not homed)");
}

void handleSeqStatus() {
  addCORS();
  String body = "{";
  body += "\"playing\":" + String(sequencePlaying ? "true" : "false") + ",";
  body += "\"index\":" + String(sequenceIndex) + ",";
  body += "\"len\":" + String(sequenceLen) + ",";
  body += "\"looping\":" + String(sequenceLooping ? "true" : "false") + ",";
  body += "\"pushes\":" + String(sequencePushes ? "true" : "false") + ",";
  body += "\"dwellMs\":" + String(sequenceDwellMs) + ",";
  body += "\"slots\":[";
  for (int i = 0; i < sequenceLen; i++) {
    if (i > 0) body += ",";
    body += String(sequenceSlots[i]);
  }
  body += "]}";
  server.send(200, "application/json", body);
}

// ---------------------------------------------------------------------------
// Dashboard
// ---------------------------------------------------------------------------

void handleRoot() {
  addCORS();
  String html = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>IRL Home Screen Rail</title>
<style>
body{font-family:sans-serif;background:#111;color:#eee;text-align:center;padding:16px;margin:0}
h2{margin-bottom:4px}
section{max-width:720px;margin:20px auto;padding:16px;background:#1b1b1b;border-radius:12px}
h3{margin-top:0;color:#9d9}
button{font-size:15px;padding:9px 14px;margin:3px;border-radius:8px;border:none;background:#2a6;color:#fff;cursor:pointer}
button:active{background:#194}
button:disabled{background:#333;color:#777;cursor:default}
button.danger{background:#a52}
button.danger:active{background:#731}
button.small{font-size:13px;padding:6px 10px}
input[type=range]{width:90%}
input[type=number]{width:90px;font-size:15px;padding:4px}
label{display:block;margin:10px 0 2px;color:#aaa;font-size:13px}
#status{margin-top:10px;font-size:14px;color:#9f9}
#homeBanner{font-weight:bold;padding:8px;border-radius:8px;margin:8px auto;max-width:720px}
.ok{background:#152;color:#bfb}
.bad{background:#611;color:#fbb}
.row{display:flex;justify-content:center;gap:6px;flex-wrap:wrap;align-items:center}
table{width:100%;border-collapse:collapse;font-size:13px}
td,th{padding:5px 4px;border-bottom:1px solid #2a2a2a;text-align:left}
th{color:#888;font-weight:normal}
td.num{text-align:right;font-variant-numeric:tabular-nums}
tr.unset td{color:#777}
tr.inert td{opacity:.5}
.pill{display:inline-block;padding:1px 7px;border-radius:9px;font-size:11px;background:#357}
.pill.b{background:#735}
button.arm{background:#2a2a2a;color:#888;min-width:30px;padding:4px 8px;margin:1px}
button.arm.on{background:#357;color:#fff}
button.arm.b.on{background:#735}
.warn{font-size:12px;color:#c96}
</style></head><body>
<h2>IRL Home Screen - rail pusher</h2>
<div id="homeBanner" class="bad">homing state unknown</div>
<div id="status">loading...</div>

<section>
<h3>Homing</h3>
<p style="font-size:12px;color:#888">Nothing moves until the rail is homed. Fast approach, back off, slow re-approach - the slow touch is what sets zero.</p>
<div class="row">
<button onclick="rehome()">HOME</button>
<button class="danger" onclick="stopAll()">STOP</button>
<button id="engageBtn" onclick="toggleEnable()">DISENGAGE</button>
</div>
<div id="endstopLine" style="margin-top:8px;font-size:13px;color:#aaa">endstop: ?</div>
</section>

<section>
<h3>Position</h3>
<input type="range" id="slider" min="0" max="12000" value="0" oninput="onSlide(this.value)">
<div id="cmLabel">0.0 cm</div>
<div class="row">
<button onclick="jogCustom(-1)">&larr; jog</button>
<input type="number" id="jogStep" value="200" min="1">
<button onclick="jogCustom(1)">jog &rarr;</button>
</div>
<div class="row">
<button class="danger" onclick="zero()">ZERO HERE</button>
<button class="danger" onclick="setEnd()">SET RAIL END HERE</button>
</div>
</section>

<section>
<h3>Slot Calibration</h3>
<p style="font-size:12px;color:#888">Pick the arm for the slot first, then jog the carriage until that arm lines up with the case and press SAVE.
The arm's offset along the carriage is baked into the saved value, so switching a slot's arm clears its position and it has to be re-taught.</p>
<div class="row">
<label style="display:inline">bulk fill: A below slot</label>
<input type="number" id="splitInput" min="0" max="17" value="9">
<button class="small" onclick="setSplit()">APPLY</button>
<button class="small danger" onclick="resetSlots()">CLEAR ALL</button>
</div>
<table id="slotTable"><tbody></tbody></table>
</section>

<section>
<h3>Pusher Servos</h3>
<p style="font-size:12px;color:#888">PUSH sweeps at half speed, holds 1s, then returns. A (GPIO33) runs 180&deg;&rarr;0&deg; clockwise; B (GPIO34) runs 0&deg;&rarr;180&deg; counterclockwise. Sliders are for manual positioning.</p>
<label>Servo A <span id="pusherAPhase" class="pill">idle</span></label>
<input type="range" id="servoASlider" min="0" max="180" value="180" oninput="onServoA(this.value)">
<div id="servoALabel">180&deg;</div>
<button onclick="pushA()">PUSH A</button>
<label>Servo B <span id="pusherBPhase" class="pill b">idle</span></label>
<input type="range" id="servoBSlider" min="0" max="180" value="0" oninput="onServoB(this.value)">
<div id="servoBLabel">0&deg;</div>
<button onclick="pushB()">PUSH B</button>
</section>

<section>
<h3>Velocity &amp; Motion</h3>
<label>Max speed (steps/sec) - cruise speed once fully ramped up</label>
<input type="range" id="velSlider" min="50" max="20000" value="8000" oninput="onVelocity(this.value)" onchange="onVelocity(this.value)">
<div id="velLabel">8000 steps/sec</div>
<label>Acceleration (steps/sec&sup2;) - higher = snappier ramp, more torque demand</label>
<input type="range" id="accelSlider" min="100" max="40000" value="20000" oninput="onAccel(this.value)" onchange="onAccel(this.value)">
<div id="accelLabel">20000 steps/sec&sup2;</div>
</section>

<section>
<h3>Driver Tuning</h3>
<label>Run current (mA)</label>
<input type="range" id="currentSlider" min="200" max="1400" step="50" value="1100" oninput="onCurrent(this.value)">
<div id="currentLabel">1100 mA</div>
<label>Microsteps</label>
<select id="microstepSelect" onchange="onMicrosteps(this.value)">
<option value="1">1 (full step)</option>
<option value="2">2</option><option value="4">4</option>
<option value="8" selected>8</option><option value="16">16</option>
<option value="32">32</option><option value="64">64</option>
<option value="128">128</option><option value="256">256</option>
</select>
<div class="warn">Changing microsteps rescales steps/cm - every taught slot position and the rail length become wrong. Re-teach after changing this.</div>
<label><input type="checkbox" id="stealthCheck" checked onchange="onStealth(this.checked)"> StealthChop (quiet mode)</label>
</section>

<section>
<h3>Inputs</h3>
<div id="inputLine" style="font-size:13px;color:#aaa">lever: ?</div>
<p style="font-size:12px;color:#888">Live switch level. The latched pull is consumed by the shelf app's /spin-status poll, so a quick pull may not be visible here.</p>
</section>

<section>
<h3>Sequence Player</h3>
<div class="row" id="seqAddRow"></div>
<div id="seqList" style="margin:10px 0;color:#ccd;font-size:14px;min-height:20px">sequence: (empty)</div>
<label>Dwell at each stop (ms)</label>
<input type="number" id="dwellInput" value="1500" min="100">
<label><input type="checkbox" id="loopCheck"> Loop</label>
<label><input type="checkbox" id="pushCheck"> Eject at each stop</label>
<div class="row">
<button onclick="seqPlay()">PLAY</button>
<button onclick="seqStop()">STOP</button>
<button class="danger" onclick="seqClear()">CLEAR</button>
</div>
</section>

<script>
const STEPS_PER_CM = )HTML" + String(STEPS_PER_CM, 1) + R"HTML(;
const NUM_SLOTS = )HTML" + String(NUM_SLOTS) + R"HTML(;
let sliderBusy = false, sliderTimer = null;
let driverEnabledLocal = true;
let slotCache = [];

function jog(n){ fetch('/jog?steps='+n); }
function jogCustom(dir){ jog(dir * (parseInt(document.getElementById('jogStep').value) || 0)); }
function zero(){ fetch('/zero').then(refresh); }
function setEnd(){ fetch('/setend').then(refresh); }
function stopAll(){ fetch('/api/stop').then(refresh); }
function rehome(){ fetch('/api/home').then(refresh); }
function toggleEnable(){ fetch('/enable?on=' + (driverEnabledLocal ? 0 : 1)).then(refresh); }

function onSlide(v){
  document.getElementById('cmLabel').innerText = (v / STEPS_PER_CM).toFixed(1) + ' cm';
  sliderBusy = true;
  clearTimeout(sliderTimer);
  sliderTimer = setTimeout(() => {
    fetch('/moveto?pos=' + v).then(() => { sliderBusy = false; });
  }, 80);
}

function onVelocity(v){ document.getElementById('velLabel').innerText = v + ' steps/sec'; fetch('/speed?stepspersec=' + v); }
function onAccel(v){ document.getElementById('accelLabel').innerText = v + ' steps/sec²'; fetch('/accel?stepspersec2=' + v); }
function onCurrent(v){ document.getElementById('currentLabel').innerText = v + ' mA'; fetch('/current?ma=' + v); }
function onMicrosteps(v){ fetch('/microsteps?n=' + v); }
function onStealth(c){ fetch('/stealth?on=' + (c ? 1 : 0)); }
function onServoA(v){ document.getElementById('servoALabel').innerText = v + '°'; fetch('/servo/a?angle=' + v); }
function onServoB(v){ document.getElementById('servoBLabel').innerText = v + '°'; fetch('/servo/b?angle=' + v); }
function pushA(){ fetch('/servo/a/push'); }
function pushB(){ fetch('/servo/b/push'); }

function slotGo(i){ fetch('/api/goto?game=' + i).then(refresh); }
function slotSave(i){ fetch('/api/setslot?game=' + i).then(loadSlots); }
function slotClear(i){ fetch('/api/clearslot?game=' + i).then(loadSlots); }
function slotEject(i){ fetch('/api/select?game=' + i).then(refresh); }
function slotArm(i, arm){ fetch('/api/slotpusher?game=' + i + '&pusher=' + arm).then(loadSlots); }
function setSplit(){
  if(!confirm('Reassign every slot from this boundary? Slots that change arm lose their taught position.')) return;
  fetch('/api/split?slot=' + document.getElementById('splitInput').value).then(loadSlots);
}
function resetSlots(){ if(confirm('Forget every taught slot position?')) fetch('/api/resetslots').then(loadSlots); }

function seqAdd(i){ fetch('/seq/add?slot=' + i).then(loadSeq); }
function seqClear(){ fetch('/seq/clear').then(loadSeq); }
function seqStop(){ fetch('/seq/stop').then(loadSeq); }
function seqPlay(){
  const dwell = parseInt(document.getElementById('dwellInput').value) || 1500;
  const loop = document.getElementById('loopCheck').checked ? 1 : 0;
  const push = document.getElementById('pushCheck').checked ? 1 : 0;
  fetch('/seq/play?dwell=' + dwell + '&loop=' + loop + '&push=' + push).then(loadSeq);
}

function loadSlots(){
  fetch('/api/slots').then(r=>r.json()).then(list=>{
    slotCache = list;
    let rows = '<tr><th>#</th><th>slot</th><th>pusher</th><th class="num">steps</th><th></th></tr>';
    let addRow = '';
    for(const s of list){
      const cls = (s.set ? '' : 'unset ') + (s.ejectable ? '' : 'inert');
      rows += '<tr class="' + cls + '">'
           +  '<td class="num">' + s.i + '</td>'
           +  '<td>' + s.name + '</td>'
           +  '<td style="white-space:nowrap">'
           +    '<button class="small arm' + (s.pusher === 'A' ? ' on' : '') + '" onclick="slotArm(' + s.i + ',\'a\')">A</button>'
           +    '<button class="small arm b' + (s.pusher === 'B' ? ' on' : '') + '" onclick="slotArm(' + s.i + ',\'b\')">B</button>'
           +  '</td>'
           +  '<td class="num">' + (s.set ? s.step : '-') + '</td>'
           +  '<td style="text-align:right;white-space:nowrap">'
           +    '<button class="small" onclick="slotGo(' + s.i + ')"' + (s.set ? '' : ' disabled') + '>GO</button>'
           +    '<button class="small" onclick="slotSave(' + s.i + ')">SAVE</button>'
           +    '<button class="small" onclick="slotEject(' + s.i + ')"' + (s.set && s.ejectable ? '' : ' disabled') + '>EJECT</button>'
           +    '<button class="small danger" onclick="slotClear(' + s.i + ')"' + (s.set ? '' : ' disabled') + '>X</button>'
           +  '</td></tr>';
      if(s.set) addRow += '<button class="small" onclick="seqAdd(' + s.i + ')">+ ' + s.i + '</button>';
    }
    document.querySelector('#slotTable tbody').innerHTML = rows;
    document.getElementById('seqAddRow').innerHTML = addRow || '<span style="color:#777;font-size:13px">teach a slot first</span>';
  });
}

function loadSeq(){
  fetch('/seq/status').then(r=>r.json()).then(s=>{
    let txt = s.len === 0 ? 'sequence: (empty)' : 'sequence: ' + s.slots.join(' → ');
    if (s.playing) txt += '  [PLAYING step ' + (s.index+1) + '/' + s.len + (s.looping?' loop':'') + (s.pushes?' +eject':'') + ']';
    document.getElementById('seqList').innerText = txt;
  });
}

function refresh(){
  fetch('/api/status').then(r=>r.json()).then(d=>{
    driverEnabledLocal = d.enabled;
    document.getElementById('engageBtn').innerText = d.enabled ? 'DISENGAGE' : 'ENGAGE';

    const banner = document.getElementById('homeBanner');
    banner.className = d.homed ? 'ok' : 'bad';
    banner.innerText = d.homed
      ? 'HOMED - rail live, 0..' + d.railMaxSteps + ' steps'
      : 'NOT HOMED - all motion refused until you press HOME';

    document.getElementById('status').innerText =
      'pos ' + d.pos + ' / target ' + d.target + ' | ' + (d.moving ? 'moving' : 'stopped')
      + ' | select: ' + d.selectPhase + (d.selectSlot >= 0 ? ' slot ' + d.selectSlot : '')
      + ' | ' + (d.enabled ? 'engaged' : 'DISENGAGED');
    document.getElementById('endstopLine').innerText = 'endstop: ' + (d.endstop ? 'TRIGGERED' : 'clear');
    document.getElementById('inputLine').innerText =
      'lever: ' + (d.lever ? 'PULLED' : 'rest');
    document.getElementById('pusherAPhase').innerText = d.pusherA;
    document.getElementById('pusherBPhase').innerText = d.pusherB;
    document.getElementById('splitInput').value = d.splitSlot;

    document.getElementById('slider').max = d.railMaxSteps;
    if (!sliderBusy) {
      document.getElementById('slider').value = d.pos;
      document.getElementById('cmLabel').innerText = (d.pos / STEPS_PER_CM).toFixed(1) + ' cm';
    }
    document.getElementById('servoASlider').value = d.servoA;
    document.getElementById('servoALabel').innerText = d.servoA + '°';
    document.getElementById('servoBSlider').value = d.servoB;
    document.getElementById('servoBLabel').innerText = d.servoB + '°';

  }).catch(()=>{});
  loadSeq();
}

document.addEventListener('keydown', e=>{
  if(e.target.tagName === 'INPUT') return;
  if(e.key==='ArrowLeft') jog(e.shiftKey ? -200 : -20);
  if(e.key==='ArrowRight') jog(e.shiftKey ? 200 : 20);
});

loadSlots();
setInterval(refresh, 400);
refresh();
</script>
</body></html>
)HTML";
  server.send(200, "text/html", html);
}

// ---------------------------------------------------------------------------
// WiFi
// ---------------------------------------------------------------------------

// Whether the post-failure scan has already run. The scan blocks for a couple of
// seconds and connectWiFi() is retried from loop() every 30 s, so scanning on
// every retry would leave the board permanently stalled on a network that is
// simply absent. Declared here, ABOVE connectWiFi(), because the Arduino
// preprocessor generates function prototypes but does not hoist variables.
bool wifiDiagDone = false;

const char* wifiStatusName(int s) {
  switch (s) {
    case WL_IDLE_STATUS:     return "IDLE";
    case WL_NO_SSID_AVAIL:   return "NO_SSID_AVAIL (radio never saw this SSID)";
    case WL_SCAN_COMPLETED:  return "SCAN_COMPLETED";
    case WL_CONNECTED:       return "CONNECTED";
    case WL_CONNECT_FAILED:  return "CONNECT_FAILED (rejected - password or MAC filter)";
    case WL_CONNECTION_LOST: return "CONNECTION_LOST";
    case WL_DISCONNECTED:    return "DISCONNECTED";
    default:                 return "unknown";
  }
}

// Dump what this board's own radio can see. This is the measurement that splits
// the problem in half: if the target SSID appears here, the antenna and the
// 2.4 GHz front end are working and the failure is authentication or a MAC
// filter. If it is absent while a phone standing in the same place sees it, the
// problem is RF or supply, and no amount of credential-checking will help.
//
// Deliberately uses only WiFi.status() and the scan API rather than the
// disconnect-reason event: the reason code lives at a different struct member
// across ESP32 core versions.
const char* resetReasonName(int r) {
  switch (r) {
    case ESP_RST_POWERON:  return "POWERON";
    case ESP_RST_EXT:      return "EXT_PIN";
    case ESP_RST_SW:       return "SOFTWARE";
    case ESP_RST_PANIC:    return "PANIC";
    case ESP_RST_INT_WDT:  return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT:      return "WDT";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case -1:               return "none recorded";
    default:               return "other";
  }
}

// Called once, early in setup(), before the step timer exists - so these NVS
// writes need no flash-critical bracketing.
void blackBoxBoot() {
  prevResetReason = prefs.getInt("bb_rst", -1);
  prevWifiOk      = prefs.getBool("bb_ok", false);
  prevWifiTries   = prefs.getInt("bb_try", 0);
  prevWifiStatus  = prefs.getInt("bb_st", -1);
  prevWifiRssi    = prefs.getInt("bb_rssi", 0);
  prevWifiAtMs    = prefs.getUInt("bb_ms", 0);

  bbResetReason = (int)esp_reset_reason();
  bbBoots = prefs.getUInt("bb_boots", 0) + 1;
  prefs.putUInt("bb_boots", bbBoots);
  prefs.putInt("bb_rst", bbResetReason);
  prefs.putBool("bb_ok", false);
  prefs.putInt("bb_try", 0);
  prefs.putInt("bb_st", -1);
  prefs.putInt("bb_rssi", 0);
  prefs.putUInt("bb_ms", 0);
}

// Record the outcome of a join attempt. Skipped while a motor is turning, for
// the usual reason: the write stops the step timer.
void blackBoxWifi() {
  if (!motorsIdle()) return;
  beginFlashCritical();
  prefs.putBool("bb_ok", wifiUp);
  prefs.putInt("bb_try", bbWifiTries);
  prefs.putInt("bb_st", (int)WiFi.status());
  prefs.putInt("bb_rssi", wifiUp ? (int)WiFi.RSSI() : wifiTargetRssi);
  prefs.putUInt("bb_ms", (uint32_t)millis());
  endFlashCritical();
}

void wifiDiagnose() {
  int st = WiFi.status();
  Serial.printf(">> WIFI: status=%d %s\n", st, wifiStatusName(st));
  Serial.print(">> WIFI: this board's MAC is ");
  Serial.println(WiFi.macAddress()); // to look it up in the router's client/filter list

  WiFi.disconnect(); // scanning while an association is still pending is unreliable
  delay(100);

  int n = WiFi.scanNetworks();
  if (n <= 0) {
    Serial.println(">> WIFI: scan found NOTHING - suspect antenna or supply, not the AP");
    wifiTargetRssi = 1;
    return;
  }
  wifiTargetRssi = 0;
  Serial.printf(">> WIFI: scan found %d networks\n", n);
  for (int i = 0; i < n; i++) {
    Serial.printf("   %-28s rssi %4d  ch %2d  enc %d%s\n",
                  WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i),
                  (int)WiFi.encryptionType(i),
                  WiFi.SSID(i) == WIFI_SSID ? "   <-- target" : "");
    if (WiFi.SSID(i) == WIFI_SSID && (wifiTargetRssi == 0 || WiFi.RSSI(i) > wifiTargetRssi)) {
      wifiTargetRssi = WiFi.RSSI(i);
    }
  }
  WiFi.scanDelete();
}

// One association attempt. `bssid` NULL means "any AP for this SSID". Returns
// true if it reached WL_CONNECTED before `timeoutMs`.
bool wifiJoin(const uint8_t* bssid, int32_t channel, unsigned long timeoutMs) {
  WiFi.disconnect(true); // drop any half-open association from a previous attempt
  delay(100);

  if (bssid) {
    Serial.printf(">> WIFI: joining %s via %02X:%02X:%02X:%02X:%02X:%02X ch %d",
                  WIFI_SSID, bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5],
                  (int)channel);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD, channel, bssid);
  } else {
    Serial.printf(">> WIFI: joining %s via any AP", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }

  WiFi.setTxPower(WIFI_TX_POWER); // again after begin(): the cap must hold for the join itself

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  return WiFi.status() == WL_CONNECTED;
}

void connectWiFi() {
  lastWifiAttemptMs = millis();
  WiFi.mode(WIFI_STA);
  // Modem sleep OFF. In station mode the radio otherwise naps between the access
  // point's beacons, and every request to the board waits for the next one -
  // tens of milliseconds typically, with one-second outliers when a TCP SYN
  // lands during a nap and has to be retransmitted. Costs some steady current,
  // which is the right trade for a mains-powered shelf.
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_TX_POWER);
  bbWifiTries++;
  if (!WiFi.config(STATIC_IP, GATEWAY_IP, SUBNET_MASK, GATEWAY_IP)) {
    Serial.println(">> WIFI: static IP config rejected");
  }
#ifdef WIFI_BSSID
  // Preferred AP first, then whatever answers. See PREFERRED_BSSID.
  if (!wifiJoin(PREFERRED_BSSID, PREFERRED_CHANNEL, WIFI_PINNED_TIMEOUT_MS)) {
    Serial.println(">> WIFI: preferred AP did not answer, falling back to any BSSID");
    wifiJoin(NULL, 0, WIFI_CONNECT_TIMEOUT_MS);
  }
#else
  wifiJoin(NULL, 0, WIFI_CONNECT_TIMEOUT_MS);
#endif

  wifiUp = (WiFi.status() == WL_CONNECTED);
  if (wifiUp) {
    Serial.print(">> WIFI: up at ");
    Serial.println(WiFi.localIP());
    // Which AP, not just which network name - see PREFERRED_BSSID.
    Serial.print(">> WIFI: via BSSID ");
    Serial.print(WiFi.BSSIDstr());
    Serial.printf("  ch %d  rssi %d\n", WiFi.channel(), WiFi.RSSI());
    Serial.print(">> WIFI: mac ");
    Serial.print(WiFi.macAddress());
    Serial.print("  mask ");
    Serial.print(WiFi.subnetMask());
    Serial.print("  gw ");
    Serial.println(WiFi.gatewayIP());
  } else {
    // Not fatal. Serial control and the whole motion stack still work, and the
    // retry in loop() picks it up later - a hard hang here is how the board ends
    // up looking dead when it is only the AP that is missing.
    Serial.println(">> WIFI: FAILED (2.4GHz only on the S2 - check the SSID's band)");
    if (!wifiDiagDone) {
      wifiDiagDone = true; // once only - see the note on the flag
      wifiDiagnose();
    }
  }
  blackBoxWifi();
}

// ---------------------------------------------------------------------------
// setup / loop
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_ENDSTOP, INPUT_PULLUP);
  pinMode(PIN_LEVER, INPUT_PULLUP);

  enableDriver(false);
  digitalWrite(PIN_STEP, LOW);
  digitalWrite(PIN_DIR, LOW);

  servoA.setPeriodHertz(50);
  servoB.setPeriodHertz(50);
  servoA.attach(PIN_SERVO_A, 500, 2400);
  servoB.attach(PIN_SERVO_B, 500, 2400);
  servoA.write(servoAAngle);
  servoB.write(servoBAngle);

  loadSettings();
  blackBoxBoot();

  TMCSerial.begin(115200, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);
  driver.begin();
  uint8_t result = driver.test_connection();
  Serial.print(">> TMC2209 UART test_connection(): ");
  Serial.println(result == 0 ? "OK" : "FAILED (check PDN_UART wiring and motor power)");

  driver.toff(4);
  applyRunCurrent(currentRunCurrentMa);
  applyMicrosteps(currentMicrosteps);
  driver.pwm_autoscale(true);
  driver.en_spreadCycle(false);

  stepper.setMaxSpeed(cruiseSpeed);
  stepper.setAcceleration(accelStepsPerSec2);
  stepper.setCurrentPosition(0);
  stepper.setMinPulseWidth(3);

  // Hardware timer ISR drives stepper.run() at 20kHz (every 50us), independent
  // of WiFi/WebServer load. This is what actually lets high step rates hold up.
  motionTimer = timerBegin(0, 80, true);        // 80MHz APB / 80 = 1MHz tick (1us/tick)
  timerAttachInterrupt(motionTimer, &onMotionTimer, true);
  timerAlarmWrite(motionTimer, 50, true);       // fire every 50 ticks = 50us = 20kHz
  timerAlarmEnable(motionTimer);

  // Seed the debounce from the real pin state so a switch that happens to be
  // held at boot doesn't register as a fresh pull the first time loop() runs.
  leverInput.raw = leverInput.settled = (digitalRead(PIN_LEVER) == LOW);

  // Home before WiFi comes up - nothing else is running yet, so blocking is free.
  homeAxis();

  connectWiFi();

  server.on("/", handleRoot);

  // shelf app
  server.on("/api/status", handleStatus);
  server.on("/api/select", handleSelect);
  server.on("/api/retract", handleRetract);
  server.on("/api/goto", handleGoto);
  server.on("/api/home", handleHome);
  server.on("/api/stop", handleStop);
  server.on("/api/testpush", handleTestPush);
  server.on("/api/slots", handleSlotsList);
  server.on("/api/setslot", handleSetSlot);
  server.on("/api/clearslot", handleClearSlot);
  server.on("/api/resetslots", handleResetSlots);
  server.on("/api/slotpusher", handleSlotPusher);
  server.on("/api/split", handleSplit);
  server.on("/api/speed", handleSpeed);
  server.on("/api/accel", handleAccel);
  server.on("/spin-status", handleSpinStatus);

  // bench dashboard
  server.on("/status", handleStatus);
  server.on("/jog", handleJog);
  server.on("/moveto", handleMoveTo);
  server.on("/zero", handleZero);
  server.on("/setend", handleSetEnd);
  server.on("/home", handleHome);
  server.on("/servo/a", handleServoA);
  server.on("/servo/b", handleServoB);
  server.on("/servo/a/push", handlePushA);
  server.on("/servo/b/push", handlePushB);
  server.on("/speed", handleSpeed);
  server.on("/accel", handleAccel);
  server.on("/enable", handleEnable);
  server.on("/current", handleCurrent);
  server.on("/microsteps", handleMicrosteps);
  server.on("/stealth", handleStealth);
  server.on("/seq/add", handleSeqAdd);
  server.on("/seq/clear", handleSeqClear);
  server.on("/seq/play", handleSeqPlay);
  server.on("/seq/stop", handleSeqStop);
  server.on("/seq/status", handleSeqStatus);

  server.begin();

  enableDriver(true);
  Serial.println(">> Ready. 'K' lists the slot table, '?' prints status.");
  printSlotTable();
}

void loop() {
  server.handleClient();
  pollSerial();
  serviceLatchedInput(leverInput);
  serviceSelect();
  serviceSequence();
  servicePushers();
  flushMotionSettings(); // deferred NVS write - only lands while the rail is stopped

  if (!wifiUp && millis() - lastWifiAttemptMs > WIFI_RETRY_INTERVAL_MS) {
    if (WiFi.status() == WL_CONNECTED) {
      wifiUp = true;
      Serial.print(">> WIFI: reconnected at ");
      Serial.println(WiFi.localIP());
    } else {
      connectWiFi();
    }
  }
}
