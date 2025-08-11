# QwstPad C++ Library for Arduino (ESP32-S3 TFT)

🚀 A lightweight C++ port of the QwstPad library designed for use with Arduino and tested on an Adafruit Feather ESP32-S3 TFT board. 
This library enables seamless communication with multiple QwstPad boards over I²C, handling button input, LED control, and logic inversion with precision. The original of this port is the Pimoroni qwstpad-micropython library [repo](https://github.com/pimoroni/qwstpad-micropython/tree/main?tab=readme-ov-file)

## 📦 Features

- Supports multiple QwstPad boards via I²C
- Active-low and active-high logic handling
- Button state tracking with bitfields
- LED control with active-low logic
- Clean serial output for debugging
- Modular and extensible C++ class design

## 🧰 Hardware Requirements

- Adafruit Feather ESP32-S3 TFT
- QwstPad boards (with TCA9555 I/O expanders)
- I²C wiring between Feather and QwstPads
- Optional: MAX17048 battery monitor
(more info below)

## 📁 Repository Structure

```
example
  ├── Feather_QwstPad_test.ino # Arduino sketch for testing 
src
  ├── qwstpad.h # QwstPad class header 
  ├── qwstpad.cpp # QwstPad class implementation 
doc
  ├── serial_output.txt
images
  ├── hardware_used.png
├── README.md # This file
    LICENSE
```
Link to image of used [hardware](https://github.com/PaulskPt/qwstpad-arduino/blob/main/images/hardware_used.jpg)

## 🔧 Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/paulskpt/qwstpad-arduino.git

2. Open Feather_QwstPad_test.ino in the Arduino IDE or VSCode with PlatformIO.

3. Connect your QwstPad boards via I²C.

4. Upload the sketch to your Feather ESP32-S3 TFT.

## 🧪 Example output
```
✅ Found MAX17048 with Chip ID: 0xC
Scanning I2C bus...
I2C device found at 0x21
I2C device found at 0x23
Pad 1, I2C address: 0x21, is connected
Pad 2, I2C address: 0x23, is connected
handleButtonPress_v2(): Pad 1, UP button pressed
blink_a_led(): Pad 1, blinking all LEDs
```
## 📚 Documentation 

QwstPad::init() — Initializes pad with I²C address

QwstPad::update() — Reads button states and applies logic

QwstPad::setLogicType() — Sets logic inversion mode

getButtonBitfield() — Returns current button state as bitfield

Note: If you want a fancy output of button press info:
Set the parameter to ```true``` in line 
```
  413 padLogic[i].buttons = pads[i]->getButtonBitfield(true);
```
then you get this output: 
```
Pad 1, key: U -> Bit index: 1, is pressed
  +-------+-------+-------+
  | PadL  | LEDs  | PadR  |
  +-------+-------+-------+
  |b15~b11|b10~b6 | b5~b0 |
  +-------+-------+-------+
  | 00000 | 00000 | 00001 | <- Pad 1 bitfield
  +-------+-------+-------+
```

## 🛠️ Development notes

This port was carefully debugged to resolve logic inversion issues and ensure consistent behavior across multiple pads. The code is modular and ready for extension — whether you want to add debounce logic, event callbacks, or integrate with other peripherals. In this moment a sort of debounce method is used by a polling timing in the loop() function of the sketch, set in:
```
391 const unsigned long pollInterval = 300;  // 300 milliseconds
```
and in:
```
397  if (start || currentTime - lastPollTime >= pollInterval) {
```

## 📜 License

MIT License — feel free to use, modify, and share.

## 🙌 Acknowledgements 

Special thanks to the original QwstPad project and the Pimoroni and Adafruit communities for their excellent hardware and documentation.

## Images 

## Hardware used

- Adafruit Feather ESP32-S3 TFT [info](https://www.adafruit.com/product/5483?srsltid=AfmBOoqu3pTaP28ehaMM7YCZ2IrkUCpeSIhgyfL7kuX6tprso31CxoPy)
- Pimoroni Qw/ST Pad (I2C Game Controller) [info](https://shop.pimoroni.com/products/qwst-pad?variant=53514400596347)
- 4 Pin JST-SH Cable (Qwiic, STEMMA QT, Qw/ST 200mm [info](https://shop.pimoroni.com/products/jst-sh-cable-qwiic-stemma-qt-compatible?variant=31910609813587)

