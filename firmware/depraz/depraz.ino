/*

  First attempt as created by ChatGPT. Interesting to see how this works. If it even compile...
  
  PMW3389 ΔX/ΔY -> Quadrature Generator (STM32 Maple core)

  - SPI1 @ PB3/PB4/PB5, NCS=PA15
  - JTAG disabled, SWD enabled (AFIO remap) to free PA15
  - Motion hint on PA0, NRESET on PA1
  - Quadrature out: PB6=X1(A), PB7=X2(B), PB8=Y1(A), PB9=Y2(B)
  - Debug UART1 on PA9/PA10 (enabled via DEBUG_SERIAL macro)
  - LED PC13 activity

  IMPORTANT:
  * PMW3389 typically needs a sensor-specific init + SROM download. This sketch
    focuses on SPI I/O and quadrature conversion. Fill in your init in sensorInit().
  * Register addresses match common PixArt maps; verify with your PMW3389 datasheet.

  License: Public Domain / CC0
*/

#include <Arduino.h>
#include <SPI.h>

// ---- Maple/libmaple include to remap SWJ (disable JTAG, keep SWD) ----
extern "C" {
  #include <libmaple/afio.h>
}

// ======================= Compile-time configuration =======================

// Compile-time debug control:
// 1 = enable Serial1 on PA9/PA10 with logs, 0 = no serial (saves time/space)
#define DEBUG_SERIAL 1

// Sampling rate (Hz) for reading ΔX/ΔY
#define SAMPLE_HZ           100
// Quadrature step service rate (Hz). Higher = smoother streams of pulses.
// ~20 kHz is a nice default that won't stress GPIO too hard.
#define STEP_HZ             20000

// SPI clock for PMW3389. Many PixArt parts use MODE3 and <= 2MHz for register IO.
// Verify and adjust per datasheet.
#define PMW_SPI_HZ          2000000
#define PMW_SPI_MODE        SPI_MODE3

// Some boards have PC13 LED active-low. Set to 1 if your LED is active-low.
#define LED_ACTIVE_LOW      1

// Optional: Maximum pulses we'll drain per step tick for each axis to prevent starvation.
// 1 = strict interleaving; larger can reduce latency at the expense of the other axis.
#define MAX_STEPS_PER_TICK_PER_AXIS  1

// =========================== Pin assignments ==============================

// SPI1 pins are fixed by hardware; we only set CS manually.
static const uint8_t PIN_CS      = PA15;   // NCS -> PMW3389
static const uint8_t PIN_MOTION  = PA0;    // MOTION from sensor
static const uint8_t PIN_NRESET  = PA1;    // NRESET to sensor

// Quadrature outputs
static const uint8_t PIN_X1 = PB6; // X channel A
static const uint8_t PIN_X2 = PB7; // X channel B
static const uint8_t PIN_Y1 = PB8; // Y channel A
static const uint8_t PIN_Y2 = PB9; // Y channel B

// LED
static const uint8_t PIN_LED = PC13;

// ================== PMW3389 likely register addresses =====================
// !!! Verify in *your* PMW3389 datasheet !!!
#define REG_MOTION      0x02
#define REG_DELTA_X_L   0x03
#define REG_DELTA_X_H   0x04
#define REG_DELTA_Y_L   0x05
#define REG_DELTA_Y_H   0x06

// ========================= Globals / State ================================

// Pending quadrature "counts" to emit. Positive => forward, Negative => reverse.
volatile int32_t x_queue = 0;
volatile int32_t y_queue = 0;

// Quadrature state machines (0..3) using Gray sequence:
// index -> (A,B) : 0:(0,0), 1:(0,1), 2:(1,1), 3:(1,0)
// Increasing index = "forward" direction. Decreasing = reverse.
volatile uint8_t qx_state = 0;
volatile uint8_t qy_state = 0;

// Timing (cooperative scheduling using micros())
uint32_t next_sample_us = 0;
uint32_t next_step_us   = 0;

// Cached for speed (computed in setup)
uint32_t sample_interval_us = 0;
uint32_t step_interval_us   = 0;

// Motion hint from PA0 (set by ISR)
volatile bool motion_flag = false;

// ========================= Utility: LED control ============================

// Helper to set LED in a board-safe way (many STM32 boards use active-low on PC13)
inline void ledWrite(bool on) {
#if LED_ACTIVE_LOW
  digitalWrite(PIN_LED, on ? LOW : HIGH);
#else
  digitalWrite(PIN_LED, on ? HIGH : LOW);
#endif
}

// ========================= SPI low-level I/O ==============================

inline void csLow()  { digitalWrite(PIN_CS, LOW); }
inline void csHigh() { digitalWrite(PIN_CS, HIGH); }

/*
  sensorWrite(reg, val)
  ---------------------
  Writes a single PMW register.
  - Many PixArt sensors require MSB=1 for writes.
  - Inter-byte timing (tSCLK-NCS etc.) kept conservative via small delays.
*/
void sensorWrite(uint8_t reg, uint8_t value) {
  SPI.beginTransaction(SPISettings(PMW_SPI_HZ, MSBFIRST, PMW_SPI_MODE));
  csLow();
  // For PixArt, writes usually set MSB=1. Adjust if your datasheet differs.
  SPI.transfer(reg | 0x80);
  SPI.transfer(value);
  // Respect tSCLK-NCS write timing. Conservative short delay.
  delayMicroseconds(1);
  csHigh();
  SPI.endTransaction();
  // Datasheet often requires a wait after writes (tSWW/tSWR). Be conservative.
  delayMicroseconds(20);
}

/*
  sensorRead(reg)
  ---------------
  Reads a single PMW register.
  - Many PixArt sensors use MSB=0 for reads.
  - tSRAD (read access delay) handled with small delay before data phase.
*/
uint8_t sensorRead(uint8_t reg) {
  SPI.beginTransaction(SPISettings(PMW_SPI_HZ, MSBFIRST, PMW_SPI_MODE));
  csLow();
  SPI.transfer(reg & 0x7F);       // MSB=0 for read (typical)
  // Wait for tSRAD (spec dependent). 160ns–1us typical. Use 1us to be safe.
  delayMicroseconds(1);
  uint8_t val = SPI.transfer(0x00);
  csHigh();
  SPI.endTransaction();
  // tSCLK-NCS read spacing; be conservative
  delayMicroseconds(1);
  return val;
}

/*
  readDelta16(loAddr, hiAddr)
  ---------------------------
  Reads a 16-bit signed delta value (two's complement) from given L/H registers.
  NOTE: Some sensors require you to read MOTION first to latch deltas.
*/
int16_t readDelta16(uint8_t loAddr, uint8_t hiAddr) {
  // Order L then H follows common PixArt conventions; verify if needed.
  uint8_t lo = sensorRead(loAddr);
  uint8_t hi = sensorRead(hiAddr);
  int16_t v  = (int16_t)((hi << 8) | lo);
  return v;
}

// ====================== Sensor helpers (reset/init) =======================

/*
  sensorHardwareReset()
  ---------------------
  Drives NRESET low briefly to reset the sensor. Timing conservative.
*/
void sensorHardwareReset() {
  pinMode(PIN_NRESET, OUTPUT);
  digitalWrite(PIN_NRESET, LOW);
  delay(10);                       // >= 10 ms low (conservative)
  digitalWrite(PIN_NRESET, HIGH);
  delay(50);                       // wait for sensor to boot
}

/*
  sensorInit()
  ------------
  Placeholder for the PMW3389 init & SROM download sequence.
  You MUST fill this per your NDA/datasheet to get valid motion data.
*/
void sensorInit() {
  // TODO: Insert your PMW3389 bring-up here:
  //  - Power-up reset write(s)
  //  - SROM download
  //  - CPI/DPI setup
  //  - Angle-snap, rest modes, frame rates, etc.
  // For now we just read MOTION once to clear latched flags.
  (void)sensorRead(REG_MOTION);
}

// ===================== Motion hint interrupt (optional) ===================

void IRAM_ATTR onMotionRise() {
  // Set a flag to hint activity (read loop will still run at 100Hz).
  motion_flag = true;
}

// ===================== Quadrature output primitives =======================

/*
  writeQuadState(pinA, pinB, state)
  ---------------------------------
  Applies Gray-coded state (0..3) to (A,B) pins:
    0 -> 0,0
    1 -> 0,1
    2 -> 1,1
    3 -> 1,0
*/
inline void writeQuadState(uint8_t pinA, uint8_t pinB, uint8_t state) {
  // Lookup tables for A/B outputs
  static const uint8_t A_lut[4] = {0,0,1,1};
  static const uint8_t B_lut[4] = {0,1,1,0};
  digitalWrite(pinA, A_lut[state]);
  digitalWrite(pinB, B_lut[state]);
}

/*
  quadStepOne(stateRef, pinA, pinB, dir)
  --------------------------------------
  Advances one Gray-code step in 'dir' (+1 forward, -1 reverse).
  Updates the pins immediately.
*/
inline void quadStepOne(volatile uint8_t* stateRef, uint8_t pinA, uint8_t pinB, int dir) {
  uint8_t s = *stateRef;
  if (dir > 0) {
    s = (s + 1) & 0x3;     // forward
  } else {
    s = (s + 3) & 0x3;     // reverse (minus 1 mod 4)
  }
  *stateRef = s;
  writeQuadState(pinA, pinB, s);
}

/*
  drainSteps()
  ------------
  Called at high rate (STEP_HZ). Emits at most MAX_STEPS_PER_TICK_PER_AXIS
  steps from each axis' queue to generate smooth quadrature.
*/
void drainSteps() {
  // X axis
  int emitted = 0;
  while (emitted < MAX_STEPS_PER_TICK_PER_AXIS) {
    noInterrupts(); // protect read-modify on x_queue
    int32_t q = x_queue;
    interrupts();
    if (q == 0) break;

    if (q > 0) {
      quadStepOne(&qx_state, PIN_X1, PIN_X2, +1);
      noInterrupts();
      x_queue--;
      interrupts();
    } else {
      quadStepOne(&qx_state, PIN_X1, PIN_X2, -1);
      noInterrupts();
      x_queue++;
      interrupts();
    }
    emitted++;
  }

  // Y axis
  emitted = 0;
  while (emitted < MAX_STEPS_PER_TICK_PER_AXIS) {
    noInterrupts();
    int32_t q = y_queue;
    interrupts();
    if (q == 0) break;

    if (q > 0) {
      quadStepOne(&qy_state, PIN_Y1, PIN_Y2, +1);
      noInterrupts();
      y_queue--;
      interrupts();
    } else {
      quadStepOne(&qy_state, PIN_Y1, PIN_Y2, -1);
      noInterrupts();
      y_queue++;
      interrupts();
    }
    emitted++;
  }
}

// ====================== Sampling / read ΔX, ΔY ============================

/*
  sampleAndQueue()
  ----------------
  Reads ΔX/ΔY from the sensor and adds them to the pending quadrature queues.
  Uses MOTION as a hint to skip reads when idle, but still called at 100 Hz.
*/
void sampleAndQueue() {
  // Read MOTION to latch deltas (common requirement). If your part differs,
  // remove this.
  uint8_t motion = sensorRead(REG_MOTION);

  // If MOTION is not set and the hint pin is low, we can early-out to save SPI.
  // (Leave this in place to reduce bus traffic when idle.)
  if (((motion & 0x80) == 0) && !motion_flag) {
    return;
  }

  // Read deltas (verify order/addresses for PMW3389)
  int16_t dx = readDelta16(REG_DELTA_X_L, REG_DELTA_X_H);
  int16_t dy = readDelta16(REG_DELTA_Y_L, REG_DELTA_Y_H);

  // Clear motion hint
  motion_flag = false;

  // Queue them for quadrature emission
  if (dx != 0 || dy != 0) {
    // LED "blink" on activity
    ledWrite(true);

    noInterrupts();
    x_queue += dx;
    y_queue += dy;
    interrupts();

#if DEBUG_SERIAL
    static uint32_t lastPrint = 0;
    uint32_t now = millis();
    // Throttle debug prints to ~20 Hz to keep serial readable
    if (now - lastPrint >= 50) {
      Serial1.print("dx=");
      Serial1.print(dx);
      Serial1.print(" dy=");
      Serial1.print(dy);
      Serial1.print(" | qx=");
      Serial1.print(x_queue);
      Serial1.print(" qy=");
      Serial1.println(y_queue);
      lastPrint = now;
    }
#endif
  }
}

// ================================ setup() =================================

void setup() {
  // --- Free PA15 by disabling JTAG while keeping SWD enabled ---
  // This uses libmaple: set SWJ to "SW only"
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);

  // --- Pins ---
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH); // CS idle high

  pinMode(PIN_MOTION, INPUT_PULLDOWN);  // Use pulldown; sensor drives high on motion
  attachInterrupt(digitalPinToInterrupt(PIN_MOTION), onMotionRise, RISING);

  pinMode(PIN_NRESET, OUTPUT);
  digitalWrite(PIN_NRESET, HIGH);       // keep sensor out of reset

  pinMode(PIN_X1, OUTPUT);
  pinMode(PIN_X2, OUTPUT);
  pinMode(PIN_Y1, OUTPUT);
  pinMode(PIN_Y2, OUTPUT);

  pinMode(PIN_LED, OUTPUT);
  ledWrite(false); // LED off

  // Initialize quadrature pins to state 0 (A=0,B=0)
  writeQuadState(PIN_X1, PIN_X2, qx_state);
  writeQuadState(PIN_Y1, PIN_Y2, qy_state);

  // --- Serial (debug) ---
#if DEBUG_SERIAL
  Serial1.begin(115200);
  delay(10);
  Serial1.println("\n[PMW3389->Quadrature] Boot");
#endif

  // --- SPI ---
  SPI.begin(); // SPI1 on PB3/4/5

  // --- Sensor reset & init ---
  sensorHardwareReset();
  sensorInit();

  // --- Timers via micros()-based scheduler ---
  sample_interval_us = 1000000UL / SAMPLE_HZ;
  step_interval_us   = 1000000UL / STEP_HZ;

  uint32_t now = micros();
  next_sample_us = now + sample_interval_us;
  next_step_us   = now + step_interval_us;

#if DEBUG_SERIAL
  Serial1.print("SAMPLE_HZ=");
  Serial1.print(SAMPLE_HZ);
  Serial1.print(" STEP_HZ=");
  Serial1.print(STEP_HZ);
  Serial1.println(" (ready)");
#endif
}

// ================================= loop() =================================

void loop() {
  uint32_t now = micros();

  // High-rate stepper for quadrature outputs
  if ((int32_t)(now - next_step_us) >= 0) {
    drainSteps();
    next_step_us += step_interval_us;

    // After emitting steps, if no activity, ensure LED goes off
    if (x_queue == 0 && y_queue == 0) {
      ledWrite(false);
    }
  }

  // 100 Hz sensor sampling
  if ((int32_t)(now - next_sample_us) >= 0) {
    sampleAndQueue();
    next_sample_us += sample_interval_us;
  }

  // Optional: lightweight idle
  // (Let the loop spin; STM32 is fast enough. You can add a tiny delay if desired.)
}
