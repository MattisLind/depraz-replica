/*

  First attempt as created by ChatGPT. Interesting to see how this works. If it even compile...

  PMW3389 ΔX/ΔY -> Quadrature, with *simple init* per user's reference
  Board/core: STM32 Maple (e.g., STM32F103)
  - SPI1: PB3=SCK, PB4=MISO, PB5=MOSI
  - NCS:  PA15  (JTAG disabled, SWD kept)
  - MOTION: PA0 (input)
  - NRESET: PA1 (output)
  - Quad out: PB6=X1(A), PB7=X2(B), PB8=Y1(A), PB9=Y2(B)
  - Debug UART1: PA9/PA10 (enabled via DEBUG_SERIAL)
  - LED: PC13

  Simple init sequence implemented here:
   1) Read Product_ID (0x00), Motion (0x02), Delta_X/Y (0x03..0x06)
   2) Set Resolution_L/H (CPI/50 encoding)
   3) Write Motion (0x02) to clear/ack
   4) Enter polling loop for ΔX/ΔY

  ⚠️ Comment requirement: code is heavily commented to explain each step.
*/

#include <Arduino.h>
#include <SPI.h>

extern "C" {
  #include <libmaple/afio.h>   // For afio_cfg_debug_ports to disable JTAG but keep SWD
}

// ---------------- Compile-time configuration ----------------

// Turn serial debug prints on/off at compile time.
#define DEBUG_SERIAL 1

// Target readout rate from the sensor (Hz)
#define SAMPLE_HZ 100

// Quadrature output service rate (Hz); higher = smoother pulses
#define STEP_HZ   20000

// Set desired CPI (DPI) here; PMW3389 expects Resolution = CPI/50 in L/H regs.
#define PMW_CPI   1600   // change as you like; valid 100..16000

// SPI timing/mode (per common PMW3389 guidance)
#define PMW_SPI_HZ   2000000
#define PMW_SPI_MODE SPI_MODE3

// PC13 LED polarity (many STM32 boards are active-low on PC13)
#define LED_ACTIVE_LOW 1

// Safety: limit steps drained per tick per axis so both axes make progress
#define MAX_STEPS_PER_TICK_PER_AXIS 1

// ---------------- Pin assignments ----------------

// SPI1 pins fixed by core; manual CS pin below
static const uint8_t PIN_CS     = PA15;   // NCS
static const uint8_t PIN_MOTION = PA0;    // MOTION (input from PMW3389)
static const uint8_t PIN_NRESET = PA1;    // NRESET (output to PMW3389)

// Quadrature outputs
static const uint8_t PIN_X1 = PB6;  // X channel A
static const uint8_t PIN_X2 = PB7;  // X channel B
static const uint8_t PIN_Y1 = PB8;  // Y channel A
static const uint8_t PIN_Y2 = PB9;  // Y channel B

// LED
static const uint8_t PIN_LED = PC13;

// ---------------- PMW3389 registers (verify with your datasheet) ----------------
#define REG_Product_ID        0x00
#define REG_Revision_ID       0x01
#define REG_Motion            0x02
#define REG_Delta_X_L         0x03
#define REG_Delta_X_H         0x04
#define REG_Delta_Y_L         0x05
#define REG_Delta_Y_H         0x06
#define REG_Resolution_L      0x0E
#define REG_Resolution_H      0x0F
#define REG_Power_Up_Reset    0x3A
#define REG_Shutdown          0x3B

// ---------------- Globals for quadrature state ----------------

// Queues of pending steps to emit as quadrature pulses; positive = forward.
volatile int32_t x_queue = 0;
volatile int32_t y_queue = 0;

// 2-bit Gray-state per axis: 0:00, 1:01, 2:11, 3:10
volatile uint8_t qx_state = 0;
volatile uint8_t qy_state = 0;

// Timers (micros-based cooperative scheduling)
uint32_t next_sample_us = 0;
uint32_t next_step_us   = 0;
uint32_t sample_interval_us = 0;
uint32_t step_interval_us   = 0;

// Motion hint from PA0
volatile bool motion_flag = false;

// ---------------- Small helpers ----------------
inline void ledWrite(bool on) {
#if LED_ACTIVE_LOW
  digitalWrite(PIN_LED, on ? LOW : HIGH);
#else
  digitalWrite(PIN_LED, on ? HIGH : LOW);
#endif
}

inline void csLow()  { digitalWrite(PIN_CS, LOW); }
inline void csHigh() { digitalWrite(PIN_CS, HIGH); }

// ---------------- SPI register access (timings per public examples) ----------------

// Write 1 byte to a register (MSB=1 indicates write on PMW3389)
void sensorWrite(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(SPISettings(PMW_SPI_HZ, MSBFIRST, PMW_SPI_MODE));
  csLow();
  SPI.transfer(reg | 0x80);        // set MSB=1 for write
  SPI.transfer(val);               // write data
  delayMicroseconds(20);           // tSCLK-NCS (write), conservative
  csHigh();
  SPI.endTransaction();
  delayMicroseconds(100);          // tSWW/tSWR (datasheet ~120us), safe lower bound
}

// Read 1 byte from a register (MSB=0 indicates read)
uint8_t sensorRead(uint8_t reg) {
  SPI.beginTransaction(SPISettings(PMW_SPI_HZ, MSBFIRST, PMW_SPI_MODE));
  csLow();
  SPI.transfer(reg & 0x7F);        // MSB=0 for read
  delayMicroseconds(35);           // tSRAD (~35us typical)
  uint8_t v = SPI.transfer(0x00);  // dummy write to clock data out
  delayMicroseconds(1);            // tSCLK-NCS (read)
  csHigh();
  SPI.endTransaction();
  delayMicroseconds(19);           // (20us) - tSCLK-NCS
  return v;
}

// Read signed 16-bit delta from low/high registers (two’s complement)
int16_t readDelta16(uint8_t loAddr, uint8_t hiAddr) {
  uint8_t lo = sensorRead(loAddr);
  uint8_t hi = sensorRead(hiAddr);
  return (int16_t)((hi << 8) | lo);
}

// ---------------- Motion hint ISR ----------------
void IRAM_ATTR onMotionRise() {
  motion_flag = true;              // simple flag; sampling loop will handle it
}

// ---------------- Quadrature helpers ----------------

// Apply Gray-code state (0..3) to given A/B pins
inline void writeQuadState(uint8_t pinA, uint8_t pinB, uint8_t s) {
  static const uint8_t A_lut[4] = {0,0,1,1};
  static const uint8_t B_lut[4] = {0,1,1,0};
  digitalWrite(pinA, A_lut[s]);
  digitalWrite(pinB, B_lut[s]);
}

// Advance one step (+1 forward, -1 reverse) and update pins
inline void quadStepOne(volatile uint8_t* sref, uint8_t pinA, uint8_t pinB, int dir) {
  uint8_t s = *sref;
  s = (dir > 0) ? ((s + 1) & 0x3) : ((s + 3) & 0x3);
  *sref = s;
  writeQuadState(pinA, pinB, s);
}

// Drain up to N steps per axis at high rate to generate pulses smoothly
void drainSteps() {
  // X axis
  int emitted = 0;
  while (emitted < MAX_STEPS_PER_TICK_PER_AXIS) {
    noInterrupts();
    int32_t q = x_queue;
    interrupts();
    if (q == 0) break;

    if (q > 0) { quadStepOne(&qx_state, PIN_X1, PIN_X2, +1); noInterrupts(); x_queue--; interrupts(); }
    else       { quadStepOne(&qx_state, PIN_X1, PIN_X2, -1); noInterrupts(); x_queue++; interrupts(); }
    emitted++;
  }

  // Y axis
  emitted = 0;
  while (emitted < MAX_STEPS_PER_TICK_PER_AXIS) {
    noInterrupts();
    int32_t q = y_queue;
    interrupts();
    if (q == 0) break;

    if (q > 0) { quadStepOne(&qy_state, PIN_Y1, PIN_Y2, +1); noInterrupts(); y_queue--; interrupts(); }
    else       { quadStepOne(&qy_state, PIN_Y1, PIN_Y2, -1); noInterrupts(); y_queue++; interrupts(); }
    emitted++;
  }
}

// ---------------- Simple init sequence (as requested) ----------------

// Optional hard reset via NRESET; harmless if tied high on your board
void sensorHardwareReset() {
  pinMode(PIN_NRESET, OUTPUT);
  digitalWrite(PIN_NRESET, LOW);
  delay(10);
  digitalWrite(PIN_NRESET, HIGH);
  delay(50);
}

/*
  sensorInitSimple()
  ------------------
  Implements the *simple* init like your demo:
   - (Optionally) perform a Power_Up_Reset write (0x3A <- 0x5A) for a clean start
   - Read Product_ID (0x00), Motion (0x02), ΔX/ΔY (0x03..0x06) once
   - Set Resolution_L/H from PMW_CPI (CPI = 50 * value)
   - Write Motion (0x02) to clear/ack
*/
void sensorInitSimple() {
  // Power-up reset sequence (keeps things predictable even without SROM)
  sensorWrite(REG_Shutdown, 0xB6);    // polite shutdown
  delay(300);
  sensorWrite(REG_Power_Up_Reset, 0x5A);
  delay(50);

  // Drop/raise CS quickly to reset SPI port (mirrors common examples)
  csLow();  delayMicroseconds(40);  csHigh();  delayMicroseconds(40);

  // 1) Read product and basic motion/delta once
  uint8_t pid = sensorRead(REG_Product_ID);
  uint8_t rev = sensorRead(REG_Revision_ID);
  uint8_t mot = sensorRead(REG_Motion);       // reading MOTION often latches deltas
  int16_t dx0 = readDelta16(REG_Delta_X_L, REG_Delta_X_H);
  int16_t dy0 = readDelta16(REG_Delta_Y_L, REG_Delta_Y_H);

#if DEBUG_SERIAL
  Serial1.print("PMW3389 Product_ID=0x"); Serial1.print(pid, HEX);
  Serial1.print(" Rev=0x");               Serial1.print(rev, HEX);
  Serial1.print(" Motion=0x");            Serial1.println(mot, HEX);
  Serial1.print("Delta0 x=");             Serial1.print(dx0);
  Serial1.print(" y=");                   Serial1.println(dy0);
#endif

  // 2) Set resolution: Resolution = CPI/50, written as 16-bit value L then H
  uint16_t cpival = (uint16_t)(PMW_CPI / 50);   // per public references
  sensorWrite(REG_Resolution_L, (uint8_t)(cpival & 0xFF));
  sensorWrite(REG_Resolution_H, (uint8_t)((cpival >> 8) & 0xFF));

#if DEBUG_SERIAL
  Serial1.print("Resolution set to ~"); Serial1.print(cpival * 50);
  Serial1.println(" CPI");
#endif

  // 3) Write Motion register once to clear/ack any pending flags
  sensorWrite(REG_Motion, 0x00);

  // Read once more to clear latched state before entering main loop
  (void)sensorRead(REG_Motion);
  (void)readDelta16(REG_Delta_X_L, REG_Delta_X_H);
  (void)readDelta16(REG_Delta_Y_L, REG_Delta_Y_H);
}

// ---------------- Sampling & queuing ----------------

// Poll at SAMPLE_HZ; uses MOTION as a hint but still polls
void sampleAndQueue() {
  uint8_t motion = sensorRead(REG_Motion);                 // read/ack
  if (((motion & 0x80) == 0) && !motion_flag) return;      // no motion, skip SPI work

  int16_t dx = readDelta16(REG_Delta_X_L, REG_Delta_X_H);  // signed two’s complement
  int16_t dy = readDelta16(REG_Delta_Y_L, REG_Delta_Y_H);

  motion_flag = false;

  if (dx != 0 || dy != 0) {
    ledWrite(true);      // show activity

    noInterrupts();
    x_queue += dx;       // queue steps to be emitted as quadrature
    y_queue += dy;
    interrupts();

#if DEBUG_SERIAL
    static uint32_t lastLog = 0;
    uint32_t now = millis();
    if (now - lastLog >= 50) {     // throttle debug prints (~20 Hz)
      Serial1.print("dx="); Serial1.print(dx);
      Serial1.print(" dy="); Serial1.print(dy);
      Serial1.print(" | qx="); Serial1.print(x_queue);
      Serial1.print(" qy="); Serial1.println(y_queue);
      lastLog = now;
    }
#endif
  }
}

// ---------------- Arduino setup/loop ----------------
void setup() {
  // Free PA15 by disabling JTAG while keeping SWD enabled
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);

  // GPIO setup
  pinMode(PIN_CS, OUTPUT);         digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_MOTION, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(PIN_MOTION), onMotionRise, RISING);

  pinMode(PIN_NRESET, OUTPUT);     digitalWrite(PIN_NRESET, HIGH);
  pinMode(PIN_X1, OUTPUT);         pinMode(PIN_X2, OUTPUT);
  pinMode(PIN_Y1, OUTPUT);         pinMode(PIN_Y2, OUTPUT);
  pinMode(PIN_LED, OUTPUT);        ledWrite(false);

  // Initialize quadrature outputs to state 0 (A=0,B=0)
  writeQuadState(PIN_X1, PIN_X2, qx_state);
  writeQuadState(PIN_Y1, PIN_Y2, qy_state);

#if DEBUG_SERIAL
  Serial1.begin(115200);
  delay(10);
  Serial1.println("\n[PMW3389->Quadrature] Boot (simple init)");
#endif

  // SPI1 begin
  SPI.begin(); // PB3/PB4/PB5

  // Optional hardware reset and simple init sequence
  sensorHardwareReset();
  sensorInitSimple();

  // Scheduler setup
  sample_interval_us = 1000000UL / SAMPLE_HZ;
  step_interval_us   = 1000000UL / STEP_HZ;
  uint32_t now = micros();
  next_sample_us = now + sample_interval_us;
  next_step_us   = now + step_interval_us;

#if DEBUG_SERIAL
  Serial1.print("SAMPLE_HZ="); Serial1.print(SAMPLE_HZ);
  Serial1.print(" STEP_HZ=");  Serial1.println(STEP_HZ);
#endif
}

void loop() {
  uint32_t now = micros();

  // High-rate quadrature service
  if ((int32_t)(now - next_step_us) >= 0) {
    drainSteps();
    next_step_us += step_interval_us;
    if (x_queue == 0 && y_queue == 0) ledWrite(false);
  }

  // 100 Hz sensor sampling
  if ((int32_t)(now - next_sample_us) >= 0) {
    sampleAndQueue();
    next_sample_us += sample_interval_us;
  }
}
