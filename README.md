# QwstPad C++ Library for Pimoroni Qw/ST Pad (I2C Game Controller)

🚀 A C++ port of the QwstPad library designed for use with Arduino and tested on an Adafruit Feather ESP32-S3 TFT board. 
This library enables seamless communication with multiple QwstPad boards over I²C, handling button input, LED control, and logic inversion with precision. The original of this port is the Pimoroni qwstpad-micropython library [repo](https://github.com/pimoroni/qwstpad-micropython/tree/main?tab=readme-ov-file)

## 📦 Features

- Supports multiple Qw/ST Pad boards via I²C
- Active-low and active-high logic handling
- Button state tracking with bitfields (example 1)
- or Button state tracking with Button Events (example 2)
- LED control
- Clean serial output for debugging
- Modular and extensible C++ class design

## 🧰 Hardware Requirements

- Adafruit Feather ESP32-S3 TFT
- Pimoroni Qw/ST Pad (I2C Game controller) boards (with TCA9555 I/O expanders) (PIM 752)
- I²C wiring between Feather and QwstPads
(more info below)

## 📁 Repository Structure

```
examples
  ├── 01_Qwstpad_test.ino # Arduino sketch for testing 
  ├── 02_Qwstpad_ButtonEvents_2pad_test.ino # Arduino sketch for testing Button Events
src
  ├── qwstpad.h # QwstPad class header 
  ├── qwstpad.cpp # QwstPad class implementation 
doc
  ├── Example1_monitor_output.txt
  ├── Example1_monitor_output_2.txt
  ├── Example_2_monitor_output.txt
  ├── Example2_monitor_output_v2.txt
images
  ├── hardware_used.jpg
  ├── qwstpads_back.jpg
├── README.md # This file
    LICENSE
```
Link to images [hardware](https://github.com/PaulskPt/qwstpad-arduino/tree/main/images)

Link to Serial Monitor [output](https://github.com/PaulskPt/qwstpad-arduino/tree/main/doc)

## 🔧 Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/paulskpt/qwstpad-arduino.git

2. Open Feather_QwstPad_test.ino in the Arduino IDE v2 or VSCode with PlatformIO.

3. Connect your QwstPad boards via I²C.

4. Build and upload the sketch to your Feather ESP32-S3 TFT.

## 🧪 Example 1 output
```
QwstPad test
✅ Found MAX17048 with Chip ID: 0xC
Calling scanI2CDevices...
Scanning I2C bus...
I2C device found at 0x21
I2C device found at 0x23
I2C device found at 0x36
Scan complete. Devices found: 3
Returned from scanI2CDevices.
QwstPad::init(): Pad 1
QwstPad::init(): Pad 2
Maximum number of QwSTPads: 2
Pad 1, I2C address: 0x21, is connected
Pad 2, I2C address: 0x23, is connected
Number of connected pads: 2
Pad 1 is initialized
Pad 2 is initialized
bitfield 00000 00000 00001
handleButtonPress(): Pad 2, UP button pressed
blink_a_led(): Pad 2, blinking one LED for button: 'UP'
```
## 📚 Documentation 

QwstPad::init() — Initializes pad with I²C address

QwstPad::update() — Reads button states and applies logic

QwstPad::setLogicType() — Sets logic inversion mode

getButtonBitfield() — Returns current button state as bitfield

There are more functions in the QwstPad C++ library that we not yet use in the test sketch

Note: If you want a fancy output of button press info:
Set the parameter to ```true``` in line 
```
  412 padLogic[i].buttons = pads[i]->getButtonBitfield(true);
```
then you get this output: 
```
  +-------+-------+-------+
  | PadL  | LEDs  | PadR  |
  +-------+-------+-------+
  |b15~b11|b10~b6 | b5~b0 |
  +-------+-------+-------+
  | 00000 | 00000 | 00001 | <- Pad 1 bitfield
  +-------+-------+-------+
handleButtonPress(): Pad 1, UP button pressed
blink_a_led(): Pad 1, blinking one LED for button: 'UP'
```

## 🛠️ Development notes

### Notes about example 1

This port was carefully debugged to resolve logic inversion issues and ensure consistent behavior across multiple pads. The code is modular and ready for extension — whether you want to add debounce logic, event callbacks, or integrate with other peripherals. In this moment a sort of debounce method is used by a polling timing in the loop() function of the sketch, set in:
```
411 const unsigned long pollInterval = 300;  // 300 milliseconds
```
and in:
```
417  if (start || currentTime - lastPollTime >= pollInterval) {
```

### Notes about example 2

This example sketch also makes use of new internal features added to the QwstPad class:

QwstPad::debugPrintStates() — prints the current button states for debugging.
```
  266 // bool btnPressed = pads[i]->debugPrintStates();
```

getCurrentStates() and getPreviousStates() — expose internal state maps for advanced use.

__config->buttonPins[] — now populated to track pin assignments per button.

These additions support more flexible debugging and custom logic for advanced applications.

## Example 2 

🧪 Example: Feather_ESP32-S3_TFT_Qwstpad_ButtonEvents_test.ino

This sketch demonstrates how to use the QwstPad class to detect and handle button events (presses and releases) on a Feather ESP32-S3 with a QwstPad connected via I²C.
In the most recent version of this test I connected four external I²C devices: the two QwstPad game controllers, one Pimoroni multi-sensor-stick and a M5Stack Unit-RTC. It revealed that this was too much for one I²C bus (note that the Feather ESP32-S3 TFT also use a battery gauge and a temperature sensor that also use the I²C bus). I successfully connected the four external I²C devices to two I²C buses. The two I²C game controllers I connected to a second I²C bus.

🔧 Key Features:

Connect both QwstPad's to a SECONDARY_I2C_PORT (Wire1) (SDA1 = GPIO10, SCL1 = GPIO11).

Initializes the QwstPad and configures it for ACTIVE_HIGH logic.

Uses pads[i]->update() to refresh internal button states of each pad.

Calls pads[i]->pollEvents() to retrieve a list of ButtonEvent objects of a pad.

Prints each detected event to the Serial Monitor in the format:

Pad1, button: <key> - PRESSED
Pad1, button: <key> - RELEASED

🧠 How example 2 works:

The sketch polls the pads every 50ms.

Internally, pads[i]->update() tracks current and previous button states.

pads[i]->pollEvents() compares those states to detect transitions.

Events are returned as a std::vector<ButtonEvent>, each containing:

key: the button identifier (e.g. "UP", "MINUS")

type: either PRESSED or RELEASED

When a button has been pressed one of the four LEDs will be blink.

## 🧪 Example 2 output

```

QwstPad ButtonEvent 2 QwstPad test
✅ Adafruit Feather ESP32-S3 TFT detected
✅ Found MAX17048 with Chip ID: 0xC
Default I2C port (Wire) I2C scan: 0x23, 0x36, 0x51, 0x6A, 0x76, 
Secondary I2C port (Wire1) I2C scan: 0x21, 0x23, 
Initializing BME280...
BME280 initialized successfully.
QwstPad::init(): Initialized pad with __padID: 0 at address: 0x21
QwstPad::init(): Initialized pad with __padID: 1 at address: 0x23
Maximum number of QwSTPads: 2
Pad 1, I2C address: 0x21, padID: 0. Pad is connected
Pad 2, I2C address: 0x23, padID: 1. Pad is connected
Number of connected pads: 2
setup(): Pad 1, use_qwstpad = true
setup(): Pad 2, use_qwstpad = true
📦 Pad Configurations:
Pad ID: 0
  Logic Type: ACTIVE_HIGH
  Button Pins: 'A' → 14  'B' → 12  'D' → 4  'L' → 2  'M' → 5  'P' → 11  'R' → 3  'U' → 1  'X' → 15  'Y' → 13  
---------------------------
Pad ID: 1
  Logic Type: ACTIVE_HIGH
  Button Pins: 'A' → 14  'B' → 12  'D' → 4  'L' → 2  'M' → 5  'P' → 11  'R' → 3  'U' → 1  'X' → 15  'Y' → 13  
---------------------------
Pad ID: 2
  Logic Type: ACTIVE_HIGH
  Button Pins: (none)
---------------------------
Pad ID: 3
  Logic Type: ACTIVE_HIGH
  Button Pins: (none)
---------------------------
read_bme280_data(): Temp: 31.12 °C, Pressure: 1006.14 mBar, Altitude: 84.35 m, Humidity: 39.91 %
ckForButtonPress(): Pad 1, btnChanged = true
ckForButtonPress(): Pad 1, button state changed for: 'UP', button: PRESSED
ckForButtonPress(): Pad 1, btn_idx = 1
ckForButtonPress(): going to call blink_a_led() ...
blink_a_led(): Pad 1, LED index: 1, blinking one LED for button: 'UP'
ckForButtonPress(): Button: 1 = 'UP', pressed at time: 1319645
loop(): a button has been pressed
ckForButtonPress(): Pad 1, btnChanged = true
ckForButtonPress(): Pad 1, button state changed for: 'UP', button: RELEASED
ckForButtonPress(): Pad 2, btnChanged = true
ckForButtonPress(): Pad 2, button state changed for: 'DOWN', button: PRESSED
ckForButtonPress(): Pad 2, btn_idx = 4
ckForButtonPress(): going to call blink_a_led() ...
blink_a_led(): Pad 2, LED index: 4, blinking one LED for button: 'DOWN'
ckForButtonPress(): Button: 4 = 'DOWN', pressed at time: 1322164
loop(): a button has been pressed
read_bme280_data(): Temp: 31.92 °C, Pressure: 1006.24 mBar, Altitude: 83.49 m, Humidity: 40.12 %
ckForButtonPress(): Pad 2, btnChanged = true
ckForButtonPress(): Pad 2, button state changed for: 'DOWN', button: RELEASED

Alternative output:

Current button states:
A: 0, B: 0, D: 0, L: 0, M: 0, P: 0, R: 0, U: 0, X: 1, Y: 0, 
Previous button states:
A: 0, B: 0, D: 0, L: 0, M: 0, P: 0, R: 0, U: 0, X: 0, Y: 0, 
Pad 1, button state changed for: 'X', button: PRESSED
Current button states:
A: 0, B: 0, D: 0, L: 0, M: 0, P: 0, R: 0, U: 0, X: 0, Y: 0, 
Previous button states:
A: 0, B: 0, D: 0, L: 0, M: 0, P: 0, R: 0, U: 0, X: 0, Y: 0, 
Current button states:
A: 0, B: 0, D: 0, L: 0, M: 0, P: 0, R: 0, U: 0, X: 0, Y: 0, 
Previous button states:
A: 0, B: 0, D: 0, L: 0, M: 0, P: 0, R: 0, U: 0, X: 1, Y: 0, 
Pad 1, button state changed for: 'X', button: RELEASED

Note: if you want this alternative output uncomment lines 192, 216 and 217.

282 // bool btnPressed = pad->debugPrintStates();

309 //if (btnPressed)
310 //  delay(3000);  // Give user time to view the results
```
This sketch is ideal for building interactive applications — from gamepads to control panels — where reacting to button events is more useful than polling raw states.


## 📜 License

MIT License — feel free to use, modify, and share.

## 🙌 Acknowledgements 

Special thanks to the original QwstPad project and the Pimoroni and Adafruit communities for their excellent hardware and documentation. Thanks to Microsoft Copilot for the great assistance!

## Hardware used

- Adafruit Feather ESP32-S3 TFT [info](https://www.adafruit.com/product/5483?srsltid=AfmBOoqu3pTaP28ehaMM7YCZ2IrkUCpeSIhgyfL7kuX6tprso31CxoPy)
  also available through [Pimoroni](https://shop.pimoroni.com/products/adafruit-esp32-s3-tft-feather-4mb-flash-2mb-psram-stemma-qt?variant=40032190857299)
- 1 x (or 2x) Pimoroni Qw/ST Pad (I2C Game Controller) [info](https://shop.pimoroni.com/products/qwst-pad?variant=53514400596347)
- 2 x 4 Pin JST-SH Cable (Qwiic, STEMMA QT, Qw/ST 200mm) [info](https://shop.pimoroni.com/products/jst-sh-cable-qwiic-stemma-qt-compatible?variant=31910609813587)
- eventually: 1 x JST-SH to DuPont Pins (CAB1004)   [info](https://shop.pimoroni.com/products/jst-sh-cable-qwiic-stemma-qt-compatible?variant=31910609846355)

Note: if you are going to use two (like in this repo), three or (maximum) four Pimoroni QwstPad boards, you need to cut copper bridges on the back of the board as indicated on that side of the board, to give each board its unique I2C address. Then, in the sketch of example 1, line 50, change the maximum number of QwstPad's used accordingly:
```
  In example 1:
  50 #define CURRENT_MAX_PADS 2

  In example 2:
  104 #define CURRENT_MAX_PADS 2
```
Then in the sketch of example 1, function setup(), uncomment the following line(s):
```
  In example 1:
  294  //pads[2] = new QwstPad(DEFAULT_I2C_PORT, ALT_ADDRESS_2, true);
  295  //pads[3] = new QwstPad(DEFAULT_I2C_PORT, ALT_ADDRESS_3, true);

  In example 2:
  566 //pads[2] = new QwstPad(SECONDARY_I2C_PORT, ALT_ADDRESS_2, true);
  567 //pads[3] = new QwstPad(SECONDARY_I2C_PORT, ALT_ADDRESS_3, true);

  Note that in Example 2 the current I2C Port for pads 0 and 1 is defined as followw:
  564 pads[0] = new QwstPad(SECONDARY_I2C_PORT, DEFAULT_ADDRESS, true);
  565 pads[1] = new QwstPad(SECONDARY_I2C_PORT, ALT_ADDRESS_1, true);
```
