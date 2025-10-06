#include <Arduino.h>
#include <HomeSpan.h> //Library for Apple Homekit
#include <esp_task_wdt.h>  // Watchdog library

#define PWMPIN 18  // PWM control for the fan
#define SIGNAL_PIN 35 // Read the LED signal
#define LED_PIN 2  // On-board LED on most ESP32 boards
#define EMULATEPIN 25 // Pin used to emulate a physical button press

const int MIN_FREQ = 75;   // minimum frequency used when mapping percent->frequency
const int MAX_FREQ = 320;  // maximum frequency used when mapping percent->frequency

const int LEDCCHANNEL = 0;
const int LEDCRES = 8; // PWM resolution (bits)

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

// State variables
int lastLevel = -1;                 // last known discrete level (0..3)
int lastAppliedPowerPercent = -1;   // last percent value sent to applyPower (-1 means off)
unsigned long ignoreLevelUntil = 0; // timestamp until which level changes/logs are ignored (ms)
bool emulating = false;             // are we currently emulating a button press?
unsigned long emulateStartTime = 0; // when emulation started (ms)
unsigned long lastLogTime = 0;      // last time we logged / toggled LED (ms)
const unsigned long logInterval = 500; // logging interval (ms)
bool ledState = false;              // internal LED toggle state for heartbeat

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
// This function uses your original PWMed API calls (kept unchanged).
void applyPower(int percent) {
  if (percent < 0) {
    Serial.println("applyPower: OFF");
    ledcWrite(PWMPIN, 0);
    ledcWriteTone(PWMPIN, 0);
  } else {
    int freq = percentToFreq(percent);
    Serial.print("applyPower: ON, power % = ");
    Serial.print(percent);
    Serial.print(", freq = ");
    Serial.println(freq);

    // Keep original behavior: write full duty (255) and set tone frequency.
    ledcWrite(PWMPIN, 255);
    ledcWriteTone(PWMPIN, freq);
  }
}

// Custom HomeSpan accessory implementing an Air Purifier service
struct MyPurifier : Service::AirPurifier {
  int power = -1;                   // current internal power representation (-1 off, >=0 percent)
  SpanCharacteristic *active;
  SpanCharacteristic *rotationSpeed;

  MyPurifier()
    : Service::AirPurifier() {
    // HomeKit characteristics: Active and RotationSpeed
    active = new Characteristic::Active(0);
    rotationSpeed = new Characteristic::RotationSpeed(0);
  }

  // Called by HomeKit when user changes characteristics (toggle or speed)
  boolean update() override {
    // If we are within ignore window (e.g. right after emulation), ignore Home updates
    if (millis() < ignoreLevelUntil) return true;

    // Determine new power: if Active is off -> -1, otherwise use rotationSpeed
    int newPower = active->getNewVal() ? rotationSpeed->getNewVal() : -1;
    if (newPower != power) {
      power = newPower;
      applyPowerFromHome(power); // handle change coming from HomeKit
    }
    return true;
  }

  // Called when a physical level change is detected (0..3).
  // Translates discrete level to approximate percent and updates HomeKit state.
  void handlePhysicalLevel(int level) {
    int physPower = -1;
    switch (level) {
      case 0: physPower = -1; break;
      case 1: physPower = 10; break;
      case 2: physPower = 40; break;
      case 3: physPower = 70; break;
    }
    if (physPower != power) {
      power = physPower;
      active->setVal(power >= 0 ? 1 : 0);    // update HomeKit Active characteristic
      rotationSpeed->setVal(physPower > 0 ? physPower : 0); // update HomeKit RotationSpeed
      Serial.print("Physical level set power to ");
      Serial.println(physPower);
    }
  }

private:
  // Handle a command coming from Home (HomeKit). Keeps existing behavior.
  void applyPowerFromHome(int percent) {
    Serial.print("applyPowerFromHome: percent=");
    Serial.println(percent);

    if (percent < 0) {
      Serial.println("OFF STATE, emulating if needed");
      if (lastLevel == 3) {
        emulateButtonPress(millis());
      }
      power = -1;
    } else {
      if (lastLevel < 1) {
        Serial.println("FAN off, emulate button press");
        emulateButtonPress(millis());
      }
      power = percent;
    }

    // Call common routine to physically apply power to the motor
    applyPower(percent);
    lastAppliedPowerPercent = percent;
  }
};

MyPurifier *myPurifierInstance = nullptr;

// Start emulation: bring EMULATEPIN LOW for a short time and ignore level readings
void emulateButtonPress(unsigned long now) {
  if (!emulating) {
    Serial.println("Starting emulation - disable analog logging for 1s");
    emulating = true;
    emulateStartTime = now;
    digitalWrite(EMULATEPIN, LOW);     // start emulation (active low in original code)
    ignoreLevelUntil = now + 1000;     // ignore level changes for 1000 ms after emulation
  }
}

// Log and react to level changes. Called when measured discrete level differs from lastLevel.
void logLevelChange(int level, unsigned long now) {
  if (level != lastLevel) {
    lastLevel = level;
    myPurifierInstance->handlePhysicalLevel(lastLevel);  // notify Home accessory

    // If we are in ignore window, skip logging and physical apply
    if (now < ignoreLevelUntil) {
      return;
    }

    int power = -1;
    switch (lastLevel) {
      case 0: power = -1; break;
      case 1: power = 10; break;
      case 2: power = 40; break;
      case 3: power = 70; break;
    }

    Serial.print("LEVEL ");
    Serial.println(lastLevel);
    Serial.print("Set power to ");
    Serial.println(power);
    applyPower(power);
  }
}

// Try to (re-)initialize the task watchdog with a given timeout (ms).
esp_err_t try_reinit_wdt(uint32_t timeout_ms) {
  // Attempt deinit if available, then init with new config
  #ifdef ESP_TASK_WDT_DEINIT
  esp_err_t e = esp_task_wdt_deinit();
  Serial.print("esp_task_wdt_deinit(): "); Serial.println((int)e);
  #endif

  esp_task_wdt_config_t cfg = {};
  cfg.timeout_ms = timeout_ms;
  cfg.idle_core_mask = 0;
  return esp_task_wdt_init(&cfg);
}

void setup() {
  // Initialize LED pin and show activity while booting
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.begin(115200);
  delay(100);

  // Reinitialize WDT with 15s timeout if possible
  esp_err_t r = try_reinit_wdt(15000);
  if (r == ESP_OK) Serial.println("Reinit WDT OK");
  else Serial.print("Reinit WDT failed: "); Serial.println((int)r);

  // Add current task to watchdog
  esp_task_wdt_add(NULL);

  // Ensure fan is OFF initially
  applyPower(-1);
  delay(1000);

  // Configure input signal pin and attach interrupt to measure pulse widths
  pinMode(SIGNAL_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_PIN), signalISR, CHANGE);

  // Configure emulate pin (active low in emulation) and set it HIGH (idle)
  pinMode(EMULATEPIN, OUTPUT);
  digitalWrite(EMULATEPIN, HIGH);

  // Keep original PWM attachment call (left unchanged to preserve behavior)
  ledcAttachChannel(PWMPIN, 1000, LEDCRES, LEDCCHANNEL);

  // HomeSpan WiFi and accessory initialization (keep original settings)
  homeSpan.setWifiCredentials("YOUR_WIFI_SSID", "YOUR_WIFI_PASSWORD");
  homeSpan.setPairingCode("12345678"); //Your pair code
  homeSpan.begin(Category::AirPurifiers, "Purifier Uppatvind");

  new SpanAccessory();
  new Service::AccessoryInformation();
  new Characteristic::Identify();

  myPurifierInstance = new MyPurifier();

  // Turn off boot LED to indicate setup finished
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  // Let HomeSpan handle HomeKit tasks
  homeSpan.poll();

  unsigned long now = millis();

  // Emulation completion logic: after ~100 ms set emulate pin HIGH again
  if (emulating && (now - emulateStartTime >= 100)) {
    digitalWrite(EMULATEPIN, HIGH);
    Serial.println("Emulation ended - pin 23 HIGH");
    emulating = false;
  }

  // Periodic processing block (every logInterval ms)
  if (now - lastLogTime >= logInterval) {
    float dutyCycle = 0.0f;
    uint8_t level = 0;

    // If ISR signaled new pulse data, copy values under critical section and compute duty
    if (pulseAvailable) {
      noInterrupts();
      unsigned long highT = highDuration;
      unsigned long lowT = lowDuration;
      pulseAvailable = false;
      interrupts();

      unsigned long period = highT + lowT;
      if (period > 0) {
        dutyCycle = (highT * 100.0f) / period;
      }
      level = getLevel(dutyCycle);
    }

    // If computed level differs from last known, handle it
    if (lastLevel != level) {
      logLevelChange(level, now);
    }

    // Simple heartbeat on-board LED toggle (visible activity)
    if (ledState) {
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
    }
    ledState = !ledState;

    lastLogTime = now;

    // Reset watchdog to show the main loop is alive
    esp_task_wdt_reset();
  }
}