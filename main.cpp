// src/main.cpp — compact logs (6 fields); home OPEN (300 overshoot);
// compCall OFF: overshoot CLOSED then park at 220
// compCall ON: fully OPEN to 0 (prints "Seating OPEN..."), seed PID=0, then resume
// Mapping: 0=open, 250=closed
// Code for Controllers 1, 2, 4, and 5 Turnbull

#include <Arduino.h>
#include <PID_v1.h>
#include <avr/wdt.h>
#include <math.h>

// ---------------- Pin Definitions ----------------
#define IN1  8
#define IN2  9
#define IN3 10
#define IN4 11
#define tempSensorPin   A0
#define thermometerPin  7      // compressor call input (alias below)
#define heatingPin      12     // unchanged (input only)
const uint8_t compCallPin = thermometerPin;

// --------------- Stepper Sequence ----------------
const int stepSequence[4][4] = {
  {1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}
};

// -------------------- Globals --------------------
volatile int stepIndex = 0;
int  currentSteps = 0;           // 0 = fully OPEN, stepsMax = fully CLOSED
bool direction    = true;

// ---- ADC/divider config ----
const double vRef    = 5.0;
const double vSupply = 5.0;
const double rPullup = 10000.0;

// ---- Live captures ----
int    rawAdc  = 0;
double vNode   = 0.0;

// ---- Noise reduction / filtering ----
const uint8_t adcSamples = 32;
const uint8_t trimEach   = 6;
double filteredLnR = NAN;
const double tempTauSec = 2.5;
double filteredTempF    = NAN;

// Setpoint & steps
const double desiredTemp = 57.0;   // °F
const int stepsMax = 250;          // fully CLOSED (mechanical max)
const int stepsMin = 0;            // fully OPEN
const int maxCloseSteps = 250;     // do not close beyond this (leave crack open)
const int homeOvershootOpen  = 300;  // startup: overshoot OPEN end
const int offOvershootClose  = 300;  // compCall OFF: overshoot CLOSED end

// PID vars
const int pidSampleTime = 5000;    // ms
double inputTemp;                  // °F (filtered) fed to PID
double outputSteps;                // PID absolute target (steps)
double setpoint = desiredTemp;     // °F
double Kp = 0.5, Ki = 0.05, Kd = 0.0;
PID eevPID(&inputTemp, &outputSteps, &setpoint, Kp, Ki, Kd, DIRECT);

// Raw sensor resistance from A0 (ohms)
double sensorResistanceOhms = NAN;

// Re-init tracking
unsigned long lastReinit = 0;
const unsigned long dayInMs = 86400000UL;   // 24h

// State/latches
bool parkedClosed = false;   // true when we've overshot closed & parked at 220

// Semantics: HIGH = ON, LOW = OFF
#define ON   HIGH
#define OFF  LOW
inline bool isOn(uint8_t pin) { return digitalRead(pin) == ON; }

// Status print timer
const unsigned long statusIntervalMs = 1000;
unsigned long lastStatusMs = 0;

// ---------- Small utility: sort + trimmed mean for ints ----------
static void sortIntArray(int *a, uint8_t n) {
  for (uint8_t i = 1; i < n; ++i) { int k=a[i], j=i-1; while (j>=0 && a[j]>k){a[j+1]=a[j]; j--;} a[j+1]=k; }
}
static int readAdcTrimmedMean(uint8_t pin, uint8_t samples, uint8_t trim) {
  if (samples < 3) samples = 3; if (trim*2 >= samples) trim = (samples-1)/2;
  (void)analogRead(pin); // settle S/H
  const uint8_t N = samples; int buf[64];
  for (uint8_t i=0;i<N;++i) buf[i]=analogRead(pin);
  sortIntArray(buf,N); uint8_t s=trim,e=N-trim; long sum=0; for(uint8_t i=s;i<e;++i) sum+=buf[i];
  return (int)(sum/(e-s));
}

// -------------- Helpers --------------
static double steinhartF_from_lnR(double lnR) {
  const double A=1.009249522e-03,B=2.378405444e-04,C=2.019202697e-07;
  double tempK = 1.0/(A + B*lnR + C*lnR*lnR*lnR);
  return (tempK - 273.15) * 9.0/5.0 + 32.0;
}

void updateTemperature() {
  static unsigned long lastMs = 0;
  unsigned long now = millis();
  double dt = (lastMs==0)?0.25:(now-lastMs)/1000.0; lastMs = now;

  rawAdc = readAdcTrimmedMean(tempSensorPin, adcSamples, trimEach);
  vNode  = (rawAdc * vRef) / 1023.0;
  if (vNode <= 1e-6) vNode = 1e-6;
  if (vNode >= (vSupply-1e-6)) vNode = vSupply-1e-6;

  sensorResistanceOhms = rPullup * (vNode / (vSupply - vNode));

  double lnR = log(sensorResistanceOhms);
  if (isnan(filteredLnR)) filteredLnR = lnR;
  double alpha = dt / (tempTauSec + dt);
  filteredLnR += alpha * (lnR - filteredLnR);

  filteredTempF = steinhartF_from_lnR(filteredLnR);
  inputTemp     = filteredTempF;   // PID uses filtered temperature
}

// Stepper motion primitives
void stepMotor(volatile int step) {
  digitalWrite(IN1, stepSequence[step][0]);
  digitalWrite(IN2, stepSequence[step][1]);
  digitalWrite(IN3, stepSequence[step][2]);
  digitalWrite(IN4, stepSequence[step][3]);
}
void moveStepper(int steps, int speed) {
  if (steps==0 || speed<=0) return;
  direction = (steps>0); steps = abs(steps);
  int stepDelay = 1000 / speed;
  for (int i=0;i<steps;i++) {
    wdt_reset(); stepMotor(stepIndex);
    stepIndex = direction ? (stepIndex+1)%4 : (stepIndex+3)%4;
    delay(stepDelay);
  }
  currentSteps += direction ? steps : -steps;
  if (currentSteps > stepsMax) currentSteps = stepsMax;
  if (currentSteps < stepsMin) currentSteps = stepsMin;
}
void moveToPosition(int position, int speed) {
  if (position > stepsMax) position = stepsMax;
  if (position < stepsMin) position = stepsMin;
  int delta = position - currentSteps; moveStepper(delta, speed);
}

// --- Startup: hard home with overshoot to the OPEN end (0 steps) ---
void initializeStepper() {
  Serial.println("Homing to OPEN (overshoot 300)...");
  moveStepper(-homeOvershootOpen, 25);   // drive past OPEN end
  currentSteps = stepsMin;               // 0 = fully OPEN
  delay(200);
  Serial.println("Home complete.");
}

// --- One-shot: overshoot CLOSED then back off to 220 ---
void seatClosedOvershoot() {
  Serial.println("Seating CLOSED (overshoot)...");
  moveStepper(+offOvershootClose, 25);   // drive past CLOSED end
  currentSteps = stepsMax;               // assert mechanically fully closed
  moveToPosition(maxCloseSteps, 25);     // back off to 220
  Serial.println("Parked at 220.");
}

// --- Periodic re-home (close -> open, both with overshoot) ---
void reinitSteppers() {
  Serial.println("Re-homing steppers...");
  moveStepper(+homeOvershootOpen, 25);   // past CLOSED end
  delay(150);
  moveStepper(-homeOvershootOpen, 25);   // back past OPEN end
  currentSteps = stepsMin;               // reset to 0
  delay(150);
  Serial.println("Re-homing complete.");
}

inline void printStatusLine(bool heating_on, bool comp_call_on) {
  // 6 fields: heatingOn, compCallOn, refrigerantF, rOhm, currentSteps, pidTargetSteps
  Serial.print("log ");
  Serial.print("heatingOn:");       Serial.print(heating_on ? "ON":"OFF");
  Serial.print(" compCallOn:");     Serial.print(comp_call_on ? "ON":"OFF");
  Serial.print(" refrigerantF:");   Serial.print(filteredTempF, 1);
  Serial.print(" rOhm:");           Serial.print(sensorResistanceOhms, 0);
  Serial.print(" currentSteps:");   Serial.print(currentSteps);
  Serial.print(" pidTargetSteps:"); Serial.println(outputSteps);
}

// -------------------- Arduino Setup/Loop --------------------
void setup() {
  Serial.begin(9600);
  delay(100);
#if defined(USBCON) || defined(ARDUINO_ARCH_SAMD)
  while (!Serial) { ; }
#endif
  Serial.println(); Serial.println("=== Boot ===");

  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT); pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
  pinMode(compCallPin, INPUT_PULLUP);
  pinMode(heatingPin,  INPUT_PULLUP);

#if defined(__AVR__)
  DIDR0 |= (1 << ADC0D); // reduce ADC0 noise
#endif

  // Watchdog (8s)
  wdt_enable(WDTO_8S); wdt_reset();

  // Home first so currentSteps is valid
  initializeStepper();

  // PID init: limit to 0..220 (leave a crack open while running)
  eevPID.SetOutputLimits(stepsMin, maxCloseSteps);   // 0 .. 220
  eevPID.SetSampleTime(pidSampleTime);               // 5000 ms
  eevPID.SetMode(MANUAL);
  outputSteps = currentSteps;                        // seed to home (0)
  eevPID.SetMode(AUTOMATIC);                         // Initialize() internally
  Serial.println("PID EEV Control Initialized (DIRECT)");

  // If we boot with compCall OFF, immediately seat closed and park at 220
  if (!isOn(compCallPin)) {
    eevPID.SetMode(MANUAL);
    outputSteps = maxCloseSteps;                    // reflect in log
    seatClosedOvershoot();
    parkedClosed = true;
  }

  wdt_reset();
  lastReinit = millis();
  Serial.println("Setup complete. Entering loop...");
}

void loop() {
  wdt_reset();

  // Sample refrigerant temp / resistance frequently (filtered internally)
  static unsigned long lastTempMs = 0;
  if (millis() - lastTempMs >= 250) { updateTemperature(); lastTempMs = millis(); }

  bool heating_on   =
#if defined(DEBUG)
    false;
#else
    isOn(heatingPin);
#endif
  bool comp_call_on =
#if defined(DEBUG)
    true;
#else
    isOn(compCallPin);
#endif

  // Daily re-home
  if (millis() - lastReinit >= dayInMs) { reinitSteppers(); lastReinit = millis(); }

  // ---------------- Unified control logic ----------------
  if (!comp_call_on) {
    // Ensure we physically seat closed once, then stay parked at 220
    if (!parkedClosed) {
      eevPID.SetMode(MANUAL);              // freeze PID, avoid windup
      outputSteps = maxCloseSteps;         // 220 in log
      seatClosedOvershoot();               // run the motor
      parkedClosed = true;
    } else if (currentSteps != maxCloseSteps) {
      moveToPosition(maxCloseSteps, 25);   // enforce park at 220 if drifted
    }
  } else {
    // compCall just turned ON? Fully OPEN to 0, seed PID=0, then resume
    if (parkedClosed) {
      Serial.println("Seating OPEN (to 0)...");
      eevPID.SetMode(MANUAL);
      outputSteps = stepsMin;              // 0 in log
      if (currentSteps != stepsMin) moveToPosition(stepsMin, 25); // fully open
      Serial.println("Open at 0. PID resumed.");
      eevPID.SetMode(AUTOMATIC);           // resume PID after opening
      parkedClosed = false;
    }

    if (heating_on) {
      if (currentSteps != stepsMin) moveToPosition(stepsMin, 25); // stay fully open in heating
    } else {
      if (eevPID.Compute()) {
        int target = (int)outputSteps;     // absolute steps target (0..220)
        int delta  = target - currentSteps;
        int speed  = (abs(delta) > 100) ? 75 : (abs(delta) < 25 ? 25 : 50);
        moveToPosition(target, speed);
      }
    }
  }

  // ---- Single status line per second (6 fields) ----
  if (millis() - lastStatusMs >= statusIntervalMs) { lastStatusMs = millis(); printStatusLine(heating_on, comp_call_on); }

  wdt_reset();
}
