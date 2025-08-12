# QwstPad C++ Library for Arduino (ESP32-S3 TFT)

🚀 A lightweight C++ port of the QwstPad library designed for use with Arduino and tested on an Adafruit Feather ESP32-S3 TFT board. 
This library enables seamless communication with multiple QwstPad boards over I²C, handling button input, LED control, and logic inversion with precision. The original of this port is the Pimoroni qwstpad-micropython library [repo](https://github.com/pimoroni/qwstpad-micropython/tree/main?tab=readme-ov-file)

## 📦 Features

- Supports multiple QwstPad boards via I²C
- Active-low and active-high logic handling
- Button state tracking with bitfields (example 1)
- or Button state tracking with Button Events (example 2)
- LED control
- Clean serial output for debugging
- Modular and extensible C++ class design

## 🧰 Hardware Requirements

- Adafruit Feather ESP32-S3 TFT
- Pimoroni QwstPad (I2C Game controller) boards (with TCA9555 I/O expanders)
- I²C wiring between Feather and QwstPads
(more info below)

## 📁 Repository Structure

```
examples
  ├── 01_Qwstpad_test.ino # Arduino sketch for testing 
  ├── 02_Qwstpad_ButtonEvents_test.ino # Arduino sketch for testing Button Events
src
  ├── qwstpad.h # QwstPad class header 
  ├── qwstpad.cpp # QwstPad class implementation 
doc
  ├── Example1_monitor_output.txt
  ├── Example1_monitor_output_2.txt
  ├── Example_2_monitor_output.txt
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
✅ Found MAX17048 with Chip ID: 0xC
Scanning I2C bus...
I2C device found at 0x21
I2C device found at 0x23
Pad 1, I2C address: 0x21, is connected
Pad 2, I2C address: 0x23, is connected
handleButtonPress(): Pad 1, UP button pressed
blink_a_led(): Pad 1, blinking all LEDs
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
blink_a_led(): Pad 1, LED index: 1
blinking one LED for button ID: 1
```

## 🛠️ Development notes

### Notes about example 1

This port was carefully debugged to resolve logic inversion issues and ensure consistent behavior across multiple pads. The code is modular and ready for extension — whether you want to add debounce logic, event callbacks, or integrate with other peripherals. In this moment a sort of debounce method is used by a polling timing in the loop() function of the sketch, set in:
```
391 const unsigned long pollInterval = 300;  // 300 milliseconds
```
and in:
```
397  if (start || currentTime - lastPollTime >= pollInterval) {
```

### Notes about example 2

This example sketch also makes use of new internal features added to the QwstPad class:

QwstPad::debugPrintStates() — prints the current button states for debugging. To use this feature: uncomment line:
```
    111 // bool btnPressed = pad->debugPrintStates();
```

getCurrentStates() and getPreviousStates() — expose internal state maps for advanced use.

__config->buttonPins[] — now populated to track pin assignments per button.

These additions support more flexible debugging and custom logic for advanced applications.

## Example 2 

🧪 Example: Feather_ESP32-S3_TFT_Qwstpad_ButtonEvents_test.ino

This sketch demonstrates how to use the QwstPad class to detect and handle button events (presses and releases) on a Feather ESP32-S3 with a QwstPad connected via I²C.

🔧 Key Features:

Initializes the QwstPad and configures it for ACTIVE_HIGH logic.

Uses pad->update() to refresh internal button states.

Calls pad->pollEvents() to retrieve a list of ButtonEvent objects.

Prints each detected event to the Serial Monitor in the format:

Button: <key> - PRESSED
Button: <key> - RELEASED

🧠 How example 2 works:

The sketch polls the pad every 50ms.

Internally, update() tracks current and previous button states.

pollEvents() compares those states to detect transitions.

Events are returned as a std::vector<ButtonEvent>, each containing:

key: the button identifier (e.g. "U", "M")

type: either PRESSED or RELEASED

## 🧪 Example 2 output

```
QwstPad ButtonEvent test
✅ Found MAX17048 with Chip ID: 0xC
is connected
Address: 0x21
Button state changed for: 'MINUS', button: PRESSED
Button state changed for: 'MINUS', button: RELEASED
Button state changed for: 'PLUS', button: PRESSED
Button state changed for: 'PLUS', button: RELEASED
Button state changed for: 'U', button: PRESSED
Button state changed for: 'U', button: RELEASED
Button state changed for: 'B', button: PRESSED
Button state changed for: 'B', button: RELEASED

Alternative output:

Current button states:
A: 0, B: 0, D: 0, L: 0, M: 1, P: 0, R: 0, U: 0, X: 0, Y: 0, 
Previous button states:
A: 0, B: 0, D: 0, L: 0, M: 0, P: 0, R: 0, U: 0, X: 0, Y: 0, 
Button: M - PRESSED
Current button states:
A: 0, B: 0, D: 0, L: 0, M: 0, P: 0, R: 0, U: 0, X: 0, Y: 0, 
Previous button states:
A: 0, B: 0, D: 0, L: 0, M: 1, P: 0, R: 0, U: 0, X: 0, Y: 0, 
Button: M - RELEASED

Note: if you want this alternative output uncomment lines 111, 127 and 128.

111 // bool btnPressed = pad->debugPrintStates();

138 //if (btnPressed)
139 //  delay(3000);  // Give user time to view the results
```
This sketch is ideal for building interactive applications — from gamepads to control panels — where reacting to button events is more useful than polling raw states.


## 📜 License

MIT License — feel free to use, modify, and share.

## 🙌 Acknowledgements 

Special thanks to the original QwstPad project and the Pimoroni and Adafruit communities for their excellent hardware and documentation. Thanks to Microsoft Copilot for the great assistance!

## Hardware used

- Adafruit Feather ESP32-S3 TFT [info](https://www.adafruit.com/product/5483?srsltid=AfmBOoqu3pTaP28ehaMM7YCZ2IrkUCpeSIhgyfL7kuX6tprso31CxoPy)
  also available through [Pimoroni](https://shop.pimoroni.com/products/adafruit-esp32-s3-tft-feather-4mb-flash-2mb-psram-stemma-qt?variant=40032190857299)
- Pimoroni Qw/ST Pad (I2C Game Controller) [info](https://shop.pimoroni.com/products/qwst-pad?variant=53514400596347)
- 2 x 4 Pin JST-SH Cable (Qwiic, STEMMA QT, Qw/ST 200mm) [info](https://shop.pimoroni.com/products/jst-sh-cable-qwiic-stemma-qt-compatible?variant=31910609813587)

Note: if you are going to use two (like in this repo), three or (maximum) four Pimoroni QwstPad boards, you need to cut copper bridges on the back of the board as indicated on that side of the board, to give each board its unique I2C address. Then, in the sketch of example 1, line 50, change the maximum number of QwstPad's used accordingly:
```
  In example 1:
  50 #define CURRENT_MAX_PADS 2

  In example 2:
  43 #define CURRENT_MAX_PADS 1
```
Then in the sketch of example 1, function setup(), uncomment the following line(s):
```
  In example 1:
  294  //pads[2] = new QwstPad(DEFAULT_I2C_PORT, ALT_ADDRESS_2);
  295  //pads[3] = new QwstPad(DEFAULT_I2C_PORT, ALT_ADDRESS_3);

Note that example 2 uses only one QwstPad controller.
```
