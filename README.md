# HomeSpan IKEA UPPÅTVIND

Apple home-compatible ESP32 retrofit controller for the **IKEA UPPÅTVIND** air purifier.
This project doesn't replace the original control logic: a **HomeSpan-based ESP32** connects directly to the original board and adds
**Apple Home (HomeKit via the HomeSpan library)** support, while the original button and LED keep working.

The board simply attaches to the existing control PCB using four wires:

* Reads the LED signal (to detect current power/speed level)
* Simulates the button press (to toggle power levels)
* Controls the fan via PWM frequency

---

## ✨ Features

* Native **Apple Home** integration (via HomeSpan)
* **Reads the purifier’s LED signal** (PWM duty cycle) to detect active mode (Off / Low / Medium / High)
* **Simulates button press** for mode cycling (power & speed)
* **Controls fan speed** via PWM frequency output
* Automatically synchronizes state changes between:

    * Physical button presses
    * Home app commands
* On-board LED heartbeat (blinks to indicate activity or running state)
* Setup from the phone — without Wi-Fi the purifier starts its own access point with a one-page setup
* Hold the purifier button for 5 seconds to reset Wi-Fi and HomeKit pairing
* No web interface or cloud dependency

---

## ⚙️ Hardware Setup

### Wiring Diagram

Below is the basic wiring setup between the ESP32 dev board and the purifier control board.

```
+----------------------------------+
|        IKEA UPPÅTVIND PCB        |
|                                  |
|  [SIGNAL_PIN] ----- TP7          |
|  [EMULATEPIN] -------- TP4       |
|  [PWMPIN] - WIRE Black to fan    |
|  [GND] ----------- TP2           |
|  [VIN] ----------- TP3           |
+----------------------------------+
             |
             v
+----------------------------------+
|            ESP32 Dev Board        |
|                                  |
|  SIGNAL_PIN   = GPIO 35 (Input)  |
|  EMULATEPIN   = GPIO 25 (Output) |
|  PWMPIN       = GPIO 18 (PWM Out)|
|  LED_PIN      = GPIO 2  (Status) |
|  GND          = Common Ground    |
|  VIN          = +5v              |
+----------------------------------+
```

---

### Pin Summary

| Function              | ESP32 Pin | Direction | Description                              |
|-----------------------|-----------|-----------|------------------------------------------|
| **LED Signal Input**  | GPIO 35   | Input     | Reads LED PWM to determine current speed |
| **Button Emulation**  | GPIO 25   | Output    | Simulates a physical button press        |
| **Fan Control (PWM)** | GPIO 18   | Output    | Controls fan power by PWM frequency      |
| **Status LED**        | GPIO 2    | Output    | Indicates board is alive / operational   |
| **Ground**            | GND       | Common    | Shared reference with purifier board     |
| **+5v**               | VIN       | Input     | Get power for ESP32 board                |

### Button

The main button the UPPÅTVIND is active-low switch. It's schematic is like:

```
               S1
              _|_
U1--R1--TP4--o   o--GND
         |
         |
       ESP32
```

### LED

The main LED (LED1) of the UPPÅTVIND has 4 states, off, and low, medium, and high brightnesses. The PWM is used to
control the brightness.

The LED has similar wiring, but instead of an output pin the ESP32 uses an input pin connected to TP7.
The ESP32 can then measure the pulse width to sense the LED brightness.

```
U1--TP7--LED1--GND
     |
     |
   ESP32
```

| UPPÅTVIND | ESP32   | Notes                                                      |
|-----------|---------|------------------------------------------------------------|
| TP2       | GND     |                                                            |
| TP3       | VIN     | +5V                                                        |
| TP4       | GPIO 25 | TP4 is between the button switch at S1 and the UPPÅTVIND   |
| TP7       | GPIO 35 | TP7 is between anode (positive) side of LED1 the UPPÅTVIND |


![Board](assets/board-top.jpg)

Board connections

- Power up ESP32 board: TP3 and TP2 (TP3 is +5V, TP2 is GND)
- Button imitate press: TP4
- LED check state: TP7

---

## 🧠 Behavior

* When the **physical button** on the purifier is pressed, the ESP32 detects the change in LED PWM signal and updates
  its internal state.
* When the **Home app** changes the purifier state (On/Off or speed), the ESP32:

    * If purifier is off → simulates one physical press to power it on.
    * If purifier is turned off → simulates as many presses as needed to cycle the original board to off
      (the button cycles 1 → 2 → 3 → off).
    * Adjusts the PWM output frequency to match the desired speed.
* LED signal analysis is based on **PWM high/low timing** to infer brightness and thus speed mode.
  A new level is accepted only after two consecutive matching samples, to filter glitches.
* The **on-board LED** (GPIO 2) blinks periodically to confirm operation or watchdog activity.

---

## 📦 Installation

1. **Clone the repository**

   ```bash
   git clone https://github.com/mrSerg/homespan-ikea-uppatvind.git
   cd homespan-ikea-uppatvind
   ```

2. **Open in Arduino IDE**

    * Install the [HomeSpan](https://github.com/HomeSpan/HomeSpan) library.
    * Select the correct board: **ESP32 Dev Module**.
    * Set **Tools → Partition Scheme → Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)** — HomeSpan doesn't fit the default
      1.2MB app partition, and OTA updates need two app slots.
    * Open `homespan-ikea-uppatvind.ino`.

3. **Flash the firmware**

    * Connect the ESP32 to your computer via USB.
    * Upload the sketch.
    * Open the serial monitor (115200 baud) to verify connection.

4. **Connect to Wi-Fi**

    * On first start (or after a reset) the purifier creates the Wi-Fi network **IKEA-Uppatvind**,
      password `uppatvind`.
    * Join it from your phone — the setup page opens automatically (or go to `http://192.168.4.1`).
    * The page shows the **pairing QR code and code** — take a screenshot or write it down
      (or press *Generate new code* first).
    * Choose your home network, enter its password and press *Save and connect*. The purifier reboots
      and joins your network; the setup access point is gone until the next reset.

5. **Pair with Apple Home**

    * In the **Apple Home app** tap *Add Accessory → More options*, pick **Purifier Uppatvind**
      and enter the pairing code from the setup page, or scan the saved QR code (e.g. shown on another screen).
    * After setup, the device will automatically reconnect after reboot.

### Firmware update (OTA)

Reset the purifier (see below) to start the setup access point, open the setup page and use **Firmware update** at the
bottom. Upload the `.bin` file from **Sketch → Export Compiled Binary** in Arduino IDE. Wi-Fi settings are erased by the
reset, so connect to Wi-Fi again after the update.

### Reset

Hold the purifier button for **5 seconds**: Wi-Fi settings and HomeKit pairing are erased and the
setup access point starts again. Remove the old purifier from the Home app before adding it again.

---

## ⚡ How It Works

| Component       | Function                                                              |
|-----------------|-----------------------------------------------------------------------|
| **LED Signal**  | Provides PWM waveform representing purifier speed (OFF/LOW/MED/HIGH). |
| **PWM Control** | ESP32 outputs PWM with frequency mapping to speed levels (75–320 Hz). |
| **Button Sim.** | Digital pulse (~100 ms) emulates the purifier’s tactile button press. |
| **Watchdog**    | HomeSpan watchdog reboots the ESP32 if the firmware hangs for 15 s.   |
| **HomeSpan**    | Handles HomeKit (Apple Home) pairing and state synchronization.       |

---

## 📷 Media

![Wiring](assets/IMG_7648.jpg)
![Installation](assets/IMG_7652.jpg)
![Apple homekit sync](assets/IMG_7653.jpg)
![Apple homekit sync detailed](assets/IMG_7656.jpg)
![Apple homekit sync detailed](assets/IMG_7657.jpg)

---

## Demo videos

- [Demo Video 1](assets/IMG_7655-1.mp4)
- [Demo Video 2](assets/IMG_7655-2.mp4)
- [Demo Video 3 (yes, accent is bad, i know)](assets/IMG_7659.mp4)

---

## 🧬 Known Limitations

* Apple Home can't pass Wi-Fi credentials to the purifier (that needs Apple's MFi-only WAC), so Wi-Fi is set up via the access point first.
* The physical button only gives four discrete levels (OFF / 1 / 2 / 3); from Apple Home the speed is set smoothly (0–100%).
* PWM frequency mapping may vary slightly depending on fan hardware tolerance.
* Requires shared GND with purifier control board.

---

## 🪪 License

This project is licensed under the [MIT License](LICENSE).

---

## 💡 Credits

Inspired by

[jonathonlui/esphome-ikea-uppatvind](https://github.com/jonathonlui/esphome-ikea-uppatvind)

[maxmacstn/ESP32-IKEA-UPPATVIND](https://github.com/maxmacstn/ESP32-IKEA-UPPATVIND)

[horvathgergo/esp8266-for-uppatvind](https://github.com/horvathgergo/esp8266-for-uppatvind)

Adapted and extended with **HomeSpan (HomeKit) integration**

---

## Tags for search

- IKEA apple home
- Uppatvind smart
- Uppatvind apple
- Purifier ikea apple home
- Add Uppatvind to apple home
- Uppatvind Apple homekit