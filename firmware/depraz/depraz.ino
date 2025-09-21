/*

  First attempt as created by ChatGPT. Interesting to see how this works. If it even compile...

  PMW3389 → Quadrature bridge (init aligned with depraz demo main.c)

  Init flow (like your demo):
    1) Read ProductID
    2) Read Motion + DeltaX/DeltaY (clear state)
    3) Write resolution (CPI) to CONFIG1
    4) Write Motion register (value configurable below)
    5) Enter loop reading deltas

  Hardware (STM32, Arduino_Core_STM32):
    - SPI1: PB3=SCK, PB4=MISO, PB5=MOSI; NCS=PA15
    - MOTION=PA0 (input), NRESET=PA1 (output)
    - Quadrature out: X1=PB6, X2=PB7, Y1=PB8, Y2=PB9
    - LED: PC13 (active-low on many boards)
    - Debug UART: USART1 on PA9/PA10 (define DEBUG_SERIAL 1 to enable)
    - JTAG disabled, SWD kept enabled

  NOTE: Many PMW33xx designs require SROM upload after reset for valid motion.
        Your demo’s simple init works on your board/firmware; we follow that here.
*/

#include <SPI.h>

// ------------------------- Compile-time configuration ------------------

// Enable/disable serial debug (USART1 PA9/PA10)
#define DEBUG_SERIAL 1

// Desired resolution in CPI; PMW3389 uses 50-CPI steps via CONFIG1
#define DESIRED_CPI 1600

// Value to write to MOTION register in init step (match your demo).
// If your original code used another value, change here.
#define MOTION_WRITE_VAL 0x00

// Quadrature pulse width per half-step (µs)
#define QUAD_PULSE_US 20

// Sample rate 100 Hz = 10 ms
#define SAMPLE_PERIOD_MS 10

// ------------------------- Pin assignments -----------------------------

const uint8_t PIN_NCS    = PA15; // PMW3389 NCS
const uint8_t PIN_MOTION = PA0;  // PMW3389 MOTION (interrupt)
const uint8_t PIN_NRESET = PA1;  // PMW3389 NRESET (output)

const uint8_t PIN_X1 = PB6; // Quadrature X A
const uint8_t PIN_X2 = PB7; // Quadrature X B
const uint8_t PIN_Y1 = PB8; // Quadrature Y A
const uint8_t PIN_Y2 = PB9; // Quadrature Y B

const uint8_t PIN_LED = PC13;

// ------------------------- PMW3389 register map (subset) ---------------

// Common PMW33xx regs; names per datasheet family
#define REG_PRODUCT_ID     0x00
#define REG_REVISION_ID    0x01
#define REG_MOTION         0x02
#define REG_DELTA_X_L      0x03
#define REG_DELTA_X_H      0x04
#define REG_DELTA_Y_L      0x05
#define REG_DELTA_Y_H      0x06
#define REG_CONFIG1        0x0F  // CPI = value*50 (range 1..0xFF depending on model)

// ------------------------- Globals/state --------------------------------

volatile bool motionFlag = false;     // set in MOTION ISR
uint8_t quadStateX = 0;               // 2-bit Gray state
uint8_t quadStateY = 0;
uint32_t lastSampleMs = 0;

// ------------------------- HAL: disable JTAG, keep SWD ------------------


static void disableJTAG_keepSWD() {
  // Frigör PB3/PB4/PB5 för SPI men behåll SWD för debug/programmering
  __HAL_AFIO_REMAP_SWJ_NOJTAG();
}

// ------------------------- LED helpers ----------------------------------

inline void ledOn() {
  digitalWrite(PIN_LED, HIGH);
}

inline void ledOff() {
  digitalWrite(PIN_LED, LOW);
}

// ------------------------- SPI helpers ----------------------------------

inline void ncsLow()  { digitalWrite(PIN_NCS, LOW);  }
inline void ncsHigh() { digitalWrite(PIN_NCS, HIGH); }

// PMW3389 uses SPI mode 3
static uint8_t spiReadReg(uint8_t reg) {
  SPI.transfer(reg & 0x7F);          // MSB=0 for read
  delayMicroseconds(1);              // tSRAD (säker marginal)
  uint8_t v = SPI.transfer(0x00);
  delayMicroseconds(10);
  return v;
}

static void spiWriteReg(uint8_t reg, uint8_t val) {
  SPI.transfer(reg | 0x80);          // MSB=1 for write
  SPI.transfer(val);
  delayMicroseconds(10);             
}

// ------------------------- Sensor init (matches your demo) --------------

static void sensorInitSimple() {
  // (Valfri) hårdvarureset via NRESET om kopplad
  digitalWrite(PIN_NRESET, LOW);
  delay(100);
  digitalWrite(PIN_NRESET, HIGH);
  delay(1000);

  uint8_t pid = spiReadReg(REG_PRODUCT_ID);
  uint8_t rev = spiReadReg(REG_REVISION_ID);
#if DEBUG_SERIAL
  Serial1.print("ProductID=0x"); Serial1.print(pid, HEX);
  Serial1.print(" RevisionID=0x"); Serial1.println(rev, HEX);
#endif

  (void)spiReadReg(REG_MOTION);
  (void)spiReadReg(REG_DELTA_X_L);
  (void)spiReadReg(REG_DELTA_X_H);
  (void)spiReadReg(REG_DELTA_Y_L);
  (void)spiReadReg(REG_DELTA_Y_H);

  uint8_t cpi_val = (uint8_t)((DESIRED_CPI + 49) / 50);
  spiWriteReg(REG_CONFIG1, cpi_val);
#if DEBUG_SERIAL
  Serial1.print("CPI set to "); Serial1.print((int)cpi_val * 50);
  Serial1.print(" (CONFIG1=0x"); Serial1.print(cpi_val, HEX); Serial1.println(")");
#endif

  spiWriteReg(REG_MOTION, MOTION_WRITE_VAL);
#if DEBUG_SERIAL
  Serial1.print("Motion reg written: 0x");
  Serial1.println(MOTION_WRITE_VAL, HEX);
#endif
}


// ------------------------- Quadrature generation ------------------------

static const uint8_t graySeq[4] = { 0b00, 0b01, 0b11, 0b10 };

inline void writeXPins(uint8_t bits) {
  digitalWrite(PIN_X1, (bits & 0x01) ? HIGH : LOW); // A
  digitalWrite(PIN_X2, (bits & 0x02) ? HIGH : LOW); // B
#if DEBUG_SERIAL
  Serial1.print("X1:");
  Serial1.println(bits & 0x01, DEC);
  Serial1.print("X2:");
  Serial1.println((bits>>1) & 0x01, DEC);
#endif

}

inline void writeYPins(uint8_t bits) {
  digitalWrite(PIN_Y1, (bits & 0x01) ? HIGH : LOW); // A
  digitalWrite(PIN_Y2, (bits & 0x02) ? HIGH : LOW); // B
#if DEBUG_SERIAL
  Serial1.print("Y1:");
  Serial1.println(bits & 0x01, DEC);
  Serial1.print("Y2:");
  Serial1.println(((bits>>1) & 0x01), DEC);
#endif  
}

static void stepQuadX(int steps) {
  int n = steps;
  int dir = (n >= 0) ? +1 : -1;
  n = abs(n);
  while (n--) {
    quadStateX = (quadStateX + (dir > 0 ? 1 : 3)) & 0x03;
    writeXPins(graySeq[quadStateX]);
    delayMicroseconds(QUAD_PULSE_US);
  }
}

static void stepQuadY(int steps) {
  int n = steps;
  int dir = (n >= 0) ? +1 : -1;
  n = abs(n);
  while (n--) {
    quadStateY = (quadStateY + (dir > 0 ? 1 : 3)) & 0x03;
    writeYPins(graySeq[quadStateY]);
    delayMicroseconds(QUAD_PULSE_US);
  }
}

// ------------------------- Read deltas at 100 Hz ------------------------

static void readDeltas(int16_t &dx, int16_t &dy) {
  uint8_t motion = spiReadReg(REG_MOTION);
  uint8_t xl = spiReadReg(REG_DELTA_X_L);
  uint8_t xh = spiReadReg(REG_DELTA_X_H);
  uint8_t yl = spiReadReg(REG_DELTA_Y_L);
  uint8_t yh = spiReadReg(REG_DELTA_Y_H);

  dx = (int16_t)(((int16_t)xh << 8) | xl);
  dy = (int16_t)(((int16_t)yh << 8) | yl);

#if DEBUG_SERIAL
  // Enkel spårning av MOTION-bit för diagnos
  //Serial1.print("MOTION=0x"); Serial1.print(motion, HEX);
  //Serial1.print(" dx="); Serial1.print(dx);
  //Serial1.print(" dy="); Serial1.println(dy);
#endif
}

// ------------------------- Setup / Loop ---------------------------------

void setup() {
  disableJTAG_keepSWD();

  // LED
  pinMode(PIN_LED, OUTPUT);
  ledOff();

  // Quadrature outputs are open drain
  pinMode(PIN_X1, OUTPUT_OPEN_DRAIN);
  pinMode(PIN_X2, OUTPUT_OPEN_DRAIN);
  pinMode(PIN_Y1, OUTPUT_OPEN_DRAIN);
  pinMode(PIN_Y2, OUTPUT_OPEN_DRAIN);
  writeXPins(graySeq[quadStateX]);
  writeYPins(graySeq[quadStateY]);

  pinMode(PIN_NCS, OUTPUT);   ncsHigh();
  pinMode(PIN_NRESET, OUTPUT); digitalWrite(PIN_NRESET, HIGH);


#if DEBUG_SERIAL
  Serial1.begin(115200);
  delay(10);
  Serial1.println("\nPMW3389 → Quadrature");
#endif

  SPI.setMISO(PB4);
  SPI.setMOSI(PB5);
  SPI.setSCLK(PB3);
  SPI.setSSEL(PA15);
  SPI.begin();
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));
  sensorInitSimple();

  lastSampleMs = millis();
}

void loop() {
  uint32_t now = millis();

  if ((uint32_t)(now - lastSampleMs) >= SAMPLE_PERIOD_MS) {
    lastSampleMs += SAMPLE_PERIOD_MS;

    int16_t dx = 0, dy = 0;
    readDeltas(dx, dy);

    if (dx != 0 || dy != 0) {
      ledOn();
      stepQuadX(dx); 
      stepQuadY(dy);
      ledOff();
    } 

  }

  delay(1);
}
