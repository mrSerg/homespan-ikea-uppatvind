#include <Arduino.h>
#include <HomeSpan.h> //Library for Apple Homekit
#include "ConfigPortal.h"

#define PWMPIN 18  // PWM control for the fan
#define SIGNAL_PIN 35 // Read the LED signal
#define LED_PIN 2  // On-board LED on most ESP32 boards
#define EMULATEPIN 25 // Pin used to emulate a physical button press

const int MIN_FREQ = 75;   // minimum frequency used when mapping percent->frequency
const int MAX_FREQ = 320;  // maximum frequency used when mapping percent->frequency

const int LEDCCHANNEL = 0;
const int LEDCRES = 8; // PWM resolution (bits)

const unsigned long EMULATE_PRESS_MS = 100;  // how long the emulated button is held
const unsigned long EMULATE_GAP_MS = 300;    // pause between consecutive emulated presses
const unsigned long IGNORE_LEVEL_MS = 2000;  // must outlast two level samples (debounce)
const unsigned long FACTORY_RESET_HOLD_MS = 5000;  // hold the purifier button this long to reset Wi-Fi and pairing

// Timing variables updated by ISR to measure pulse widths
volatile unsigned long riseTime = 0;
volatile unsigned long fallTime = 0;
volatile unsigned long highDuration = 0; // high time (microseconds) of the last pulse
volatile unsigned long lowDuration = 0;  // low time (microseconds) of the last pulse
volatile bool pulseAvailable = false;    // flag set by ISR when a new pulse measurement is ready

// Mutex protecting timing variables in ISR / main loop
portMUX_TYPE isrMux = portMUX_INITIALIZER_UNLOCKED;

// Map measured duty cycle to discrete physical levels (0..3)
uint8_t getLevel(float dutyCycle) {
  if (dutyCycle < 3) return 0;
  else if (dutyCycle < 15) return 1;
  else if (dutyCycle < 50) return 2;
  else return 3;
}

// Translate discrete level (0..3) to approximate percent (-1 means off)
int levelToPower(int level) {
  switch (level) {
    case 1: return 10;
    case 2: return 40;
    case 3: return 70;
    default: return -1;
  }
}

// State variables
int lastLevel = -1;                 // last known discrete level (0..3)
int lastAppliedPowerPercent = -1;   // last percent value sent to applyPower (-1 means off)
bool ignoringLevel = false;         // are level changes / Home updates currently ignored?
unsigned long ignoreLevelStart = 0; // when the ignore window started (ms)
int pendingLevel = -1;              // level seen once, waiting for confirmation by the next sample
bool emulating = false;             // are we currently emulating a button press?
int pendingPresses = 0;             // emulated presses still to perform
unsigned long emulateStartTime = 0; // when the current press started (ms)
unsigned long emulateEndTime = 0;   // when the last press was released (ms)
unsigned long lastLogTime = 0;      // last time we logged / toggled LED (ms)
const unsigned long logInterval = 500; // logging interval (ms)
bool ledState = false;              // internal LED toggle state for heartbeat
bool portalMode = false;            // no Wi-Fi configured: serving the config portal instead of HomeKit
bool buttonHeld = false;            // is the physical button currently held?
unsigned long buttonHoldStart = 0;  // when the physical button was pressed (ms)

bool isIgnoringLevel(unsigned long now) {
  if (ignoringLevel && now - ignoreLevelStart >= IGNORE_LEVEL_MS) ignoringLevel = false;
  return ignoringLevel;
}

// Convert percent [0..100] to frequency between MIN_FREQ and MAX_FREQ
int percentToFreq(int percent) {
  if (percent < 0) return 0;
  if (percent >= 100) return MAX_FREQ;
  return MIN_FREQ + (percent * (MAX_FREQ - MIN_FREQ)) / 100;
}

// ISR to capture rising/falling edges and compute high/low durations.
// Uses gpio_get_level because SIGNAL_PIN is read in ISR context.
void IRAM_ATTR signalISR() {
  int level = gpio_get_level((gpio_num_t)SIGNAL_PIN);
  unsigned long now = micros();

  // Enter critical region for updating shared timing variables
  portENTER_CRITICAL_ISR(&isrMux);
  if (level == 1) {
    // Rising edge: lowDuration is time since last falling edge
    lowDuration = now - fallTime;
    riseTime = now;
  } else {
    // Falling edge: highDuration is time since last rising edge
    highDuration = now - riseTime;
    fallTime = now;
    pulseAvailable = true; // signal main loop that a new measurement is ready
  }
  portEXIT_CRITICAL_ISR(&isrMux);
}

// Apply power to the fan. percent < 0 => OFF.
void applyPower(int percent) {
  if (percent < 0) {
    Serial.println("applyPower: OFF");
    ledcWriteTone(PWMPIN, 0);
  } else {
    int freq = percentToFreq(percent);
    Serial.print("applyPower: ON, power % = ");
    Serial.print(percent);
    Serial.print(", freq = ");
    Serial.println(freq);
    ledcWriteTone(PWMPIN, freq);
  }
}

// Custom HomeSpan accessory implementing an Air Purifier service
struct MyPurifier : Service::AirPurifier {
  int power = -1;                   // current internal power representation (-1 off, >=0 percent)
  SpanCharacteristic *active;
  SpanCharacteristic *currentState;
  SpanCharacteristic *targetState;
  SpanCharacteristic *rotationSpeed;

  MyPurifier()
    : Service::AirPurifier() {
    // HomeKit characteristics: Active and RotationSpeed
    active = new Characteristic::Active(0);
    currentState = new Characteristic::CurrentAirPurifierState(0);  // required by HomeKit, without it Home hides speed control
    targetState = new Characteristic::TargetAirPurifierState(0);    // MANUAL
    targetState->setValidValues(1, 0);  // no air quality sensor, so hide AUTO
    rotationSpeed = new Characteristic::RotationSpeed(0);
  }

  // Called by HomeKit when user changes characteristics (toggle or speed)
  boolean update() override {
    // Reject Home updates right after emulation so Home doesn't show a state that was never applied
    if (isIgnoringLevel(millis())) return false;

    // Determine new power: if Active is off -> -1, otherwise use rotationSpeed
    int newPower = active->getNewVal() ? rotationSpeed->getNewVal() : -1;
    if (newPower != power) {
      power = newPower;
      applyPowerFromHome(power); // handle change coming from HomeKit
      syncCurrentState();
    }
    return true;
  }

  // Called when a physical level change is detected (0..3).
  // Translates discrete level to approximate percent and updates HomeKit state.
  void handlePhysicalLevel(int level) {
    int physPower = levelToPower(level);
    if (physPower != power) {
      power = physPower;
      active->setVal(power >= 0 ? 1 : 0);    // update HomeKit Active characteristic
      rotationSpeed->setVal(physPower > 0 ? physPower : 0); // update HomeKit RotationSpeed
      syncCurrentState();
      Serial.print("Physical level set power to ");
      Serial.println(physPower);
    }
  }

private:
  void syncCurrentState() {
    int state = power >= 0 ? 2 : 0;  // PURIFYING : INACTIVE
    if (currentState->getVal() != state) currentState->setVal(state);
  }

  // Handle a command coming from Home (HomeKit). Keeps existing behavior.
  void applyPowerFromHome(int percent) {
    Serial.print("applyPowerFromHome: percent=");
    Serial.println(percent);

    if (percent < 0) {
      Serial.println("OFF STATE, emulating if needed");
      if (lastLevel >= 1) {
        emulateButtonPresses(4 - lastLevel, millis());  // button cycles 1 -> 2 -> 3 -> off
      }
      power = -1;
    } else {
      if (lastLevel < 1) {
        Serial.println("FAN off, emulate button press");
        emulateButtonPresses(1, millis());
      }
      power = percent;
    }

    // Call common routine to physically apply power to the motor
    applyPower(percent);
    lastAppliedPowerPercent = percent;
  }
};

MyPurifier *myPurifierInstance = nullptr;

// Start one press: bring EMULATEPIN LOW for a short time and ignore level readings
void startButtonPress(unsigned long now) {
  Serial.println("Starting emulation - ignoring level changes");
  emulating = true;
  pendingPresses--;
  emulateStartTime = now;
  digitalWrite(EMULATEPIN, LOW);
  ignoringLevel = true;
  ignoreLevelStart = now;
}

void emulateButtonPresses(int count, unsigned long now) {
  if (emulating || pendingPresses > 0) return;
  pendingPresses = count;
  startButtonPress(now);
}

// Log and react to level changes. Called when measured discrete level differs from lastLevel.
void logLevelChange(int level, unsigned long now) {
  if (level != lastLevel) {
    lastLevel = level;

    // Level change caused by our own emulation: keep the power requested from Home
    if (isIgnoringLevel(now)) {
      return;
    }

    if (myPurifierInstance) myPurifierInstance->handlePhysicalLevel(lastLevel);  // notify Home accessory

    int power = levelToPower(lastLevel);

    Serial.print("LEVEL ");
    Serial.println(lastLevel);
    Serial.print("Set power to ");
    Serial.println(power);
    applyPower(power);
  }
}

void factoryReset() {
  Serial.println("Button held - factory reset");
  clearConfig();
  homeSpan.processSerialCommand("F");  // erases HomeKit pairing and reboots
}

// EMULATEPIN is open-drain, so while we don't drive it LOW it reads the physical button
void checkFactoryResetHold(unsigned long now) {
  if (emulating || digitalRead(EMULATEPIN) == HIGH) {
    buttonHeld = false;
    return;
  }
  if (!buttonHeld) {
    buttonHeld = true;
    buttonHoldStart = now;
  } else if (now - buttonHoldStart >= FACTORY_RESET_HOLD_MS) {
    factoryReset();
  }
}

void setup() {
  // Initialize LED pin and show activity while booting
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.begin(115200);
  delay(100);

  ledcAttachChannel(PWMPIN, 1000, LEDCRES, LEDCCHANNEL);

  // Ensure fan is OFF initially
  applyPower(-1);
  delay(1000);

  // Configure input signal pin and attach interrupt to measure pulse widths
  pinMode(SIGNAL_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_PIN), signalISR, CHANGE);

  // Open-drain like the real button: only pulls TP4 to GND, released (HIGH) when idle
  pinMode(EMULATEPIN, OUTPUT_OPEN_DRAIN | PULLUP);  // pull-up keeps the line HIGH when not wired to the purifier
  digitalWrite(EMULATEPIN, HIGH);

  DeviceConfig config = loadConfig();
  if (config.ssid.isEmpty()) {
    portalMode = true;
    startConfigPortal();
    digitalWrite(LED_PIN, LOW);
    return;
  }

  homeSpan.setWifiCredentials(config.ssid.c_str(), config.password.c_str());
  if (!config.pairingCodeApplied) {
    homeSpan.setPairingCode(config.pairingCode.c_str());  // slow (SRP), so only once after the portal
    markPairingCodeApplied();
  }
  homeSpan.enableWatchdog(15); // Reboots the ESP32 if homeSpan.poll() isn't called for 15s
  homeSpan.begin(Category::AirPurifiers, "Purifier Uppatvind");

  new SpanAccessory();
  new Service::AccessoryInformation();
  new Characteristic::Identify();

  myPurifierInstance = new MyPurifier();

  // Turn off boot LED to indicate setup finished
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  if (portalMode) {
    handleConfigPortal();
  } else {
    homeSpan.poll();
  }

  unsigned long now = millis();
  checkFactoryResetHold(now);

  // Emulation completion logic: after ~100 ms set emulate pin HIGH again
  if (emulating && (now - emulateStartTime >= EMULATE_PRESS_MS)) {
    digitalWrite(EMULATEPIN, HIGH);
    Serial.println("Emulation ended - button released");
    emulating = false;
    emulateEndTime = now;
  }

  if (!emulating && pendingPresses > 0 && (now - emulateEndTime >= EMULATE_GAP_MS)) {
    startButtonPress(now);
  }

  // Periodic processing block (every logInterval ms)
  if (now - lastLogTime >= logInterval) {
    float dutyCycle = 0.0f;
    uint8_t level = 0;

    // If ISR signaled new pulse data, copy values under critical section and compute duty
    if (pulseAvailable) {
      portENTER_CRITICAL(&isrMux);
      unsigned long highT = highDuration;
      unsigned long lowT = lowDuration;
      pulseAvailable = false;
      portEXIT_CRITICAL(&isrMux);

      unsigned long period = highT + lowT;
      if (period > 0) {
        dutyCycle = (highT * 100.0f) / period;
      }
      level = getLevel(dutyCycle);
    } else {
      // No edges: LED is either fully off or fully on (100% duty)
      level = digitalRead(SIGNAL_PIN) ? 3 : 0;
    }

    // Accept a new level only when two consecutive samples agree (filters single glitches)
    if (level != lastLevel && level == pendingLevel) {
      logLevelChange(level, now);
    }
    pendingLevel = level;

    // Simple heartbeat on-board LED toggle (visible activity)
    if (ledState) {
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
    }
    ledState = !ledState;

    lastLogTime = now;
  }
}