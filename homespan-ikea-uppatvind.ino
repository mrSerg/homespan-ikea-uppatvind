#include <Arduino.h>
#include <HomeSpan.h> //Library for Apple Homekit
#include <DHT.h>
#include "ConfigPortal.h"

#define PWMPIN 18  // PWM control for the fan
#define SIGNAL_PIN 35 // Read the LED signal
#define LED_PIN 2  // On-board LED on most ESP32 boards
#define EMULATEPIN 25 // Pin used to emulate a physical button press
#define DHT_PIN 15 // DHT11 temperature and humidity sensor data

const int MIN_FREQ = 75;   // minimum frequency used when mapping percent->frequency
const int MAX_FREQ = 320;  // maximum frequency used when mapping percent->frequency

const int LEDCCHANNEL = 0;
const int LEDCRES = 8; // PWM resolution (bits)

const unsigned long EMULATE_PRESS_MS = 100;  // how long the emulated button is held
const unsigned long EMULATE_GAP_MS = 300;    // pause between consecutive emulated presses
const unsigned long BOARD_SYNC_DELAY_MS = 3000;  // let the Home slider settle before pressing the button
const unsigned long IGNORE_LEVEL_MS = 2000;  // must outlast two level samples (debounce)
const unsigned long FACTORY_RESET_HOLD_MS = 5000;  // hold the purifier button this long to reset Wi-Fi and pairing
const unsigned long RESET_BLINK_INTERVAL_MS = 500;  // emulated press period while signalling a reset
const unsigned long CLIMATE_READ_INTERVAL_MS = 10000;  // how often temperature and humidity are sent to Home
const int CLIMATE_PROBE_ATTEMPTS = 3;
const unsigned long CLIMATE_PROBE_GAP_MS = 1100;  // DHT11 needs at least 1s between reads

// Timing variables updated by ISR to measure pulse widths
volatile unsigned long riseTime = 0;
volatile unsigned long fallTime = 0;
volatile unsigned long highDuration = 0; // high time (microseconds) of the last pulse
volatile unsigned long lowDuration = 0;  // low time (microseconds) of the last pulse
volatile bool pulseAvailable = false;    // flag set by ISR when a new pulse measurement is ready

// Mutex protecting timing variables in ISR / main loop
portMUX_TYPE isrMux = portMUX_INITIALIZER_UNLOCKED;

DHT dht(DHT_PIN, DHT11);

// Map measured duty cycle to discrete physical levels (0..3)
uint8_t getLevel(float dutyCycle) {
  if (dutyCycle < 3) return 0;
  else if (dutyCycle < 15) return 1;
  else if (dutyCycle < 50) return 2;
  else return 3;
}

int powerToLevel(int percent) {
  if (percent < 0) return 0;
  if (percent <= 33) return 1;
  if (percent <= 66) return 2;
  return 3;
}

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
int requestedLevel = -1;            // board level requested from Home, -1 when none is waiting
unsigned long levelRequestTime = 0; // when the last level was requested from Home (ms)
unsigned long emulateStartTime = 0; // when the current press started (ms)
unsigned long emulateEndTime = 0;   // when the last press was released (ms)
unsigned long lastLogTime = 0;      // last time we logged / toggled LED (ms)
const unsigned long logInterval = 500; // logging interval (ms)
bool ledState = false;              // internal LED toggle state for heartbeat
bool portalMode = false;            // no Wi-Fi configured: serving the config portal instead of HomeKit
bool buttonHeld = false;            // is the physical button currently held?
bool buttonReleasedSinceBoot = false; // has the button line been seen HIGH since power-up?
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
    // Determine new power: if Active is off -> -1, otherwise use rotationSpeed
    int newPower = active->getNewVal() ? rotationSpeed->getNewVal() : -1;
    if (newPower != power) {
      power = newPower;
      applyPowerFromHome(power); // handle change coming from HomeKit
      syncCurrentState();
    }
    return true;
  }

  void loop() override {
    // valid-values only hides AUTO in the UI; Home still writes it when the tile is tapped
    if (targetState->getVal() != 0) targetState->setVal(0);
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

  void applyPowerFromHome(int percent) {
    Serial.print("applyPowerFromHome: percent=");
    Serial.println(percent);

    pressToBoardLevel(powerToLevel(percent));
    power = percent;

    // Call common routine to physically apply power to the motor
    applyPower(percent);
    lastAppliedPowerPercent = percent;
  }
};

MyPurifier *myPurifierInstance = nullptr;

struct MyTemperatureSensor : Service::TemperatureSensor {
  SpanCharacteristic *temperature;

  MyTemperatureSensor()
    : Service::TemperatureSensor() {
    temperature = new Characteristic::CurrentTemperature(0);
  }

  void loop() override {
    if (temperature->timeVal() < CLIMATE_READ_INTERVAL_MS) return;
    float value = dht.readTemperature();  // the library caches a read for 2s, so both sensors share one measurement
    if (!isnan(value)) temperature->setVal(value);
  }
};

struct MyHumiditySensor : Service::HumiditySensor {
  SpanCharacteristic *humidity;

  MyHumiditySensor()
    : Service::HumiditySensor() {
    humidity = new Characteristic::CurrentRelativeHumidity(0);
  }

  void loop() override {
    if (humidity->timeVal() < CLIMATE_READ_INTERVAL_MS) return;
    float value = dht.readHumidity();
    if (!isnan(value)) humidity->setVal(value);
  }
};

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

void pressToBoardLevel(int target) {
  requestedLevel = target;
  levelRequestTime = millis();
}

void syncBoardLevel(unsigned long now) {
  if (requestedLevel < 0 || emulating || pendingPresses > 0 || now - levelRequestTime < BOARD_SYNC_DELAY_MS) return;
  int from = max(lastLevel, 0);
  pendingPresses = (requestedLevel - from + 4) % 4;  // button cycles off -> 1 -> 2 -> 3 -> off
  Serial.printf("Board level %d -> %d: %d press(es)\n", from, requestedLevel, pendingPresses);
  requestedLevel = -1;
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

uint8_t measureLevel() {
  // No edges: LED is either fully off or fully on (100% duty)
  if (!pulseAvailable) return digitalRead(SIGNAL_PIN) ? 3 : 0;

  portENTER_CRITICAL(&isrMux);
  unsigned long highT = highDuration;
  unsigned long lowT = lowDuration;
  pulseAvailable = false;
  portEXIT_CRITICAL(&isrMux);

  unsigned long period = highT + lowT;
  return getLevel(period > 0 ? (highT * 100.0f) / period : 0.0f);
}

void pressButtonNow() {
  digitalWrite(EMULATEPIN, LOW);
  delay(EMULATE_PRESS_MS);
  digitalWrite(EMULATEPIN, HIGH);
}

void signalFactoryReset() {
  while (digitalRead(EMULATEPIN) == LOW) delay(10);  // emulated presses only work once the real button is released
  delay(EMULATE_GAP_MS);
  pulseAvailable = false;
  delay(logInterval);

  int presses = 8 + (4 - measureLevel()) % 4;  // ~5s of blinking, ending with the original board off
  for (int i = 0; i < presses; i++) {
    pressButtonNow();
    delay(RESET_BLINK_INTERVAL_MS - EMULATE_PRESS_MS);
    homeSpan.resetWatchdog();
  }
}

void factoryReset() {
  Serial.println("Button held - factory reset");
  applyPower(-1);
  signalFactoryReset();
  clearConfig();
  homeSpan.processSerialCommand("F");  // erases HomeKit pairing and reboots
}

void checkFactoryResetHold(unsigned long now) {
  // EMULATEPIN is open-drain, so while we don't drive it LOW it reads the physical button
  if (digitalRead(EMULATEPIN) == HIGH) buttonReleasedSinceBoot = true;
  if (emulating || !buttonReleasedSinceBoot || digitalRead(EMULATEPIN) == HIGH) {  // LOW since power-up is not a press
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

bool climateSensorConnected() {
  dht.begin();
  for (int i = 0; i < CLIMATE_PROBE_ATTEMPTS; i++) {
    if (dht.read(true)) return true;
    delay(CLIMATE_PROBE_GAP_MS);
    homeSpan.resetWatchdog();
  }
  Serial.println("DHT11 not found - temperature and humidity are not exposed to Home");
  return false;
}

void setup() {
  // Initialize LED pin and show activity while booting
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.begin(115200);
  delay(100);

  homeSpan.enableWatchdog(5); // must stay 5s: a longer timeout misses the window to reboot before the purifier supply sags

  if (esp_reset_reason() == ESP_RST_POWERON) {
    WiFi.mode(WIFI_STA);  // on the purifier supply Wi-Fi only survives a software-reset boot, so the power-on boot just warms up the radio
    delay(1000);
    ESP.restart();
  }

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
  homeSpan.setWifiBegin([](const char *ssid, const char *pwd) {
    WiFi.setTxPower(WIFI_TX_POWER);
    WiFi.begin(ssid, pwd);
  });
  homeSpan.begin(Category::AirPurifiers, "Purifier Uppatvind");

  new SpanAccessory();
  new Service::AccessoryInformation();
  new Characteristic::Identify();

  myPurifierInstance = new MyPurifier();

  if (climateSensorConnected()) {
    new MyTemperatureSensor();
    new MyHumiditySensor();
  }

  // Turn off boot LED to indicate setup finished
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  if (portalMode) {
    handleConfigPortal();
    homeSpan.resetWatchdog();
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

  syncBoardLevel(now);
  if (!emulating && pendingPresses > 0 && (now - emulateEndTime >= EMULATE_GAP_MS)) {
    startButtonPress(now);
  }

  // Periodic processing block (every logInterval ms)
  if (now - lastLogTime >= logInterval) {
    uint8_t level = measureLevel();

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
