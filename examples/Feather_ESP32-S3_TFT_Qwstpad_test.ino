/*
  * Feather_ESP32-S3_TFT_Qwstpad_test.ino
  * 
  * This sketch is designed to test the QwstPad functionality with the Feather ESP32-S3 TFT.
  * It initializes the QwstPad, reads button states, and manages LED states.
  * 
  * Created by Paulus Schulinck (Github handle: @PaulskPt).
  * Date: 2025-08-10
  * License: MIT License]
  * 
  * The QwstPad library is used to interface with the Pimoroni Qw/ST Pad (I2C Game Controller) board.
  * Product info: https://shop.pimoroni.com/products/qwst-pad?variant=53514400596347
  * This library is ported by me from the Pimoroni qwstpad-micropython library.
  * See: https://github.com/pimoroni/qwstpad-micropython/blob/main/src/qwstpad.py
  * This sketch uses the C++17 language standard version.
*/
#include <Arduino.h>
#include <Wire.h>
#include "qwstpad.h"
#include "Adafruit_MAX1704X.h"
#include <Adafruit_TestBed.h>
#include <vector>
#include <utility> // for std::pair
#include <iostream>
#include <array>
#include <algorithm>

Adafruit_MAX17048 max_bat;
extern Adafruit_TestBed TB;

#define DEFAULT_I2C_PORT &Wire

struct padBtn {
  uint8_t padID = 0; // Unique identifier for the pad
  bool use_qwstpad = false;
  uint16_t address = 0;
  uint16_t buttons = 0;
  String key = "";
  uint16_t buttons_old = 0;
  int8_t logic = -1; // set to -1 if not set
  bool a_button_has_been_pressed = false;
  bool buttonPressed[NUM_BUTTONS] = {false};
  bool lastButtonState[NUM_BUTTONS] = {false};
  bool currentButtonState[NUM_BUTTONS] = {false};
  unsigned long lastDebounceTime[NUM_BUTTONS] = {0};
};

std::map<std::string, bool> buttons;

#define CURRENT_MAX_PADS 2

padBtn padLogic[CURRENT_MAX_PADS]; // Logic array aligned with pads

//uint8_t NUM_BUTTONS = 10; // defined in qwstpad.h
uint8_t NUM_PADS = 0;

QwstPad* pads[CURRENT_MAX_PADS];  // Declare globally as pointers

std::vector<std::pair<uint8_t, char>> orderedDict = {
    {15, 'X'},
    {14, 'A'},
    {13, 'Y'},
    {12, 'B'},
    {11, 'P'}, // Plus bit
    {10, ' '}, // LED bit
    {9, ' '},  // LED bit
    {8, ' '},  // Not used.
    {7, ' '},  // LED bit
    {6, ' '},  // LED bit
    {5, 'M'},  // Minus bit
    {4, 'D'},
    {3, 'R'},
    {2, 'L'},
    {1, 'U'}
};

// defined in qwstpad.h
//uint8_t LED_MAPPING[] = {0x6, 0x7, 0x9, 0xA}; // Note that 0x8 is not used.

void scanI2CDevices() {
  Serial.println("Scanning I2C bus...");
  byte count = 0;

  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at 0x");
      Serial.println(address, HEX);
      count++;
    } else if (error == 4) {
      Serial.print("Unknown error at 0x");
      Serial.println(address, HEX);
    }
  }

  if (count == 0) {
    Serial.println("No I2C devices found.");
  } else {
    Serial.print("Scan complete. Devices found: ");
    Serial.println(count);
  }
}

void blink_a_led(padBtn &padLogic, bool all_leds = false) {
  static constexpr const char txt0[] PROGMEM = "blink_a_led(): ";
  uint8_t pad_idx = padLogic.padID;
  int8_t btn_idx = pads[pad_idx]->getFirstPressedButtonBitIndex();
  uint8_t led_index = 0;

  if (btn_idx >= BUTTON_UP && btn_idx <= BUTTON_X) {
    Serial.print(txt0);
    pads[pad_idx]->pr_PadID(); // Print the pad ID

    if (!all_leds) {
      switch (btn_idx) {
        case BUTTON_UP:
          led_index = 1; // LED for UP
          break;
        case BUTTON_LEFT:
          led_index = 2; // LED for DOWN
          break;
        case BUTTON_RIGHT:
          led_index = 3; // LED for LEFT
          break;
        case BUTTON_DOWN:
          led_index = 4; // LED for RIGHT
          break;
        case BUTTON_MINUS:
          led_index = 1; // LED for MINUS
          break;
        case BUTTON_PLUS:
          led_index = 4; // LED for PLUS
          break;
        case BUTTON_X:
          led_index = 1; // LED for X
          break;
        case BUTTON_Y:
          led_index = 2; // LED for Y
          break;
        case BUTTON_A:
          led_index = 3; // LED for A
          break;
        case BUTTON_B:
          led_index = 4; // LED for B
          break;
        default:
          Serial.print(F("Invalid button index: "));
          Serial.println(btn_idx);
          return; // Invalid button index, do not proceed
      }
#ifndef MY_DEBUG
      Serial.print(F(", LED index: "));
      Serial.println(led_index, DEC);
#endif
      Serial.print(F("blinking one LED for button ID: "));
      Serial.println(btn_idx);
      pads[pad_idx]->clear_leds(); // Clear all LEDs first
      delay(500); // Keep the LEDs off for 500 ms
      pads[pad_idx]->set_led(led_index, true); // Turn on specific LED
    } else {
      Serial.println(F("blinking all LEDs"));
      pads[pad_idx]->clear_leds(); // Clear all LEDs first
      delay(500); // Keep the LEDs off for 500 ms
      pads[pad_idx]->set_leds(pads[pad_idx]->address_code());
    }
    delay(500); // Keep the LED(s) on for 500 ms
    if (all_leds)
      pads[pad_idx]->clear_leds(); // Turn off all LEDs
    else 
      pads[pad_idx]->set_led(led_index, false); // Turn off specific LED
  } else {
    Serial.print(F("Invalid button index: "));
    Serial.println(btn_idx);
    return; // Invalid button index, do not proceed
  }
}

void handleButtonPress(padBtn &padLogic, uint8_t pad_idx) {
  static constexpr const char txt0[] PROGMEM = "handleButtonPress(): ";
  static constexpr const char txt1[] PROGMEM = " button pressed";
  String key = pads[pad_idx]->getFirstPressedButtonName();
  bool found = false;

  struct ButtonLabel {
  char key;
  const char* label;
  };

  ButtonLabel labels[] = {
    {'U', "UP"},
    {'D', "DOWN"},
    {'L', "LEFT"},
    {'R', "RIGHT"},
    {'P', "PLUS"},
    {'M', "MINUS"},
    {'B', "B"},
    {'Y', "Y"},
    {'A', "A"},
    {'X', "X"}
  };
  
  if (key != "") {
#ifdef MY_DEBUG
    Serial.print(F("Button pressed: "));
    Serial.println(key);
#endif
    Serial.print(txt0);
    pads[pad_idx]->pr_PadID(); // Print the pad ID
    Serial.print(F(", "));
    
    char k = key.charAt(0);  // Get first character

    for (const auto& entry : labels) {
      if (entry.key == k) {
        Serial.print(F(entry.label));
        Serial.println(txt1);
        found = true;
        break;
      }
    }
    if (!found) {
      Serial.print(F("Unknown '"));
      Serial.print(key);
      Serial.print(F("'"));
    } else {
      blink_a_led(padLogic, false); // Blink all LEDs
    }
  }
}

void clr_buttons(uint8_t i, bool all = false) {
  if (i < 0 || i >= CURRENT_MAX_PADS) {
    Serial.println(F("Invalid pad index in clr_buttons()"));
    return;
  }

  if (all) { // Clear all pads
    for (int i = 0; i < NUM_BUTTONS; ++i) {
      padLogic[i].buttons = 0; // Clear the button state
      padLogic[i].buttons_old = 0; // Clear the old button state
      padLogic[i].a_button_has_been_pressed = false; // Reset the flag
      padLogic[i].buttonPressed[i] = 0; // Clear the button state
      padLogic[i].lastButtonState[i] = false; // Reset last button state
      padLogic[i].currentButtonState[i] = false; // Reset current button state
      padLogic[i].lastDebounceTime[i] = 0; // Reset debounce time
    }
  } else { // Clear only the specified pad
    padLogic[i].buttons = 0; // Clear the button state
    padLogic[i].buttons_old = 0; // Clear the old button state
    padLogic[i].a_button_has_been_pressed = false; // Reset the flag
    padLogic[i].buttonPressed[i] = 0; // Clear the button state
    padLogic[i].lastButtonState[i] = false; // Reset last button state
    padLogic[i].currentButtonState[i] = false; // Reset current button state
    padLogic[i].lastDebounceTime[i] = 0; // Reset debounce time
  }
}

void setup() {
  static int bootCount = 0;
  bootCount++;
  Serial.begin(115200);  // Or whatever baud rate you're using
  while (!Serial) delay(10);
  
  // esp_reset_reason_t reason = esp_reset_reason();
  // Serial.print(F("Reset cause: "));
  // Serial.println(reason);
  // Serial.print("Boot count: ");
  // Serial.println(bootCount);

  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH); // Power up I²C devices

  if (!max_bat.begin()) {
      Serial.println(F("❌ Couldn\'t find Adafruit MAX1704X?\nMake sure a battery is plugged in!"));
      while (1) delay(10);
  } else {
    Serial.print(F("✅ Found MAX17048"));
    Serial.print(F(" with Chip ID: 0x")); 
    Serial.println(max_bat.getChipID(), HEX);
  }

  Wire.begin();
  //TB.theWire = &Wire;
  //TB.printI2CBusScan(); // Optional but helpful

  Serial.println("Calling scanI2CDevices...");
  scanI2CDevices();
  Serial.println("Returned from scanI2CDevices.");

  pads[0] = new QwstPad(DEFAULT_I2C_PORT, 0x21); // DEFAULT_ADDRESS);
  pads[1] = new QwstPad(DEFAULT_I2C_PORT, 0x23); // ALT_ADDRESS_1);
  //pads[2] = new QwstPad(DEFAULT_I2C_PORT, ALT_ADDRESS_2);
  //pads[3] = new QwstPad(DEFAULT_I2C_PORT, ALT_ADDRESS_3);

  uint16_t pad1address;
  Serial.print(F("Maximum number of QwSTPads: "));
  Serial.println(CURRENT_MAX_PADS);
  for (int i = 0; i < CURRENT_MAX_PADS; ++i) {
    bool isConnected = pads[i]->isConnected();
    uint16_t pAddress = pads[i]->getAddress();
    Serial.print(F("Pad "));
    Serial.print(i+1);
    Serial.print(F(", I2C address: 0x"));
    Serial.print(pAddress, HEX);
    Serial.print(F(", "));
    padLogic[i].padID = i; //+1; // adjust for human readable 1...4
    if (isConnected) {  
      if (i == 0) {
        pad1address == pAddress;
        Serial.println(F("is connected"));
        padLogic[i].use_qwstpad = true;
        NUM_PADS++;
      }
      else if (i > 0) {
        // It happened that a pad with padID > 0 had the same address as the one with padID 0.
        // so we introduced pad1address to compare
        if (pad1address != pAddress) {
          Serial.println(F("is connected"));
          padLogic[i].address = pAddress;
          padLogic[i].use_qwstpad = true;
          NUM_PADS++;
        }
        else {
          padLogic[i].address = 0;
          padLogic[i].use_qwstpad = false;
        }
      }
    }
    else {
      Serial.println(F("is not connected"));
      padLogic[i].address = 0;
      padLogic[i].use_qwstpad = false;
    }
  }
  Serial.print("Number of connected pads: ");
  Serial.println(NUM_PADS);

  // Initialize pads
  for (int j = 0; j < NUM_PADS; ++j) {
    if (padLogic[j].use_qwstpad) {
      pads[j]->begin();
#ifdef MY_DEBUG
      Serial.print(F("Pad "));
      Serial.print(j+1);
      Serial.print(F(" initialized with address: 0x"));
      Serial.println(pads[j]->getAddress(), HEX);
#endif
    }
  }
  //delay(1000); // advised by MS Copilot
}

void loop() {
  bool  use_qwstpad;
  uint16_t padLogicLen = 0;

  for (int i = 0; i < CURRENT_MAX_PADS; i++) {
    if (!pads[i]->IsInitialized()) {
      pads[i]->begin(); // restart the pad
      // 2nd check
      if (!pads[i]->IsInitialized()) 
        padLogic[i].use_qwstpad = false;
      else 
        padLogic[i].use_qwstpad = true;
    }
    else {
      padLogic[i].use_qwstpad = true;
    }
    if (padLogic[i].use_qwstpad) {
      Serial.print(F("Pad "));
      Serial.print(i+1);
      Serial.println(F(" is initialized"));
    }
    use_qwstpad = padLogic[i].use_qwstpad;
    int8_t t_logic = pads[i]->getLogicType(); // Get logic type from QwstPad
    padLogic[i].logic = t_logic;
    if (use_qwstpad) { // = 0b 1111 1000 0011 1110
      padLogic[i].buttons_old = 0xFFFF;
    }
    else {
        padLogic[i].buttons     = 0x0;
        padLogic[i].buttons_old = 0x0;
    }
  }
  padLogicLen = sizeof(padLogic);
  std::vector<ButtonEvent> keyEvent;

  unsigned long lastPollTime = 0;
  const unsigned long pollInterval = 300;  // 300 milliseconds
  unsigned long currentTime;
  bool start = true;

while (true) {
    currentTime = millis();
    if (start || currentTime - lastPollTime >= pollInterval) {
      start = false;
      lastPollTime = currentTime;
      uint8_t btn_change_cnt = 0;
      for (uint8_t i = 0; i < NUM_PADS; i++) {
        pads[i]->clear_leds(); // Clear all LEDs
        clr_buttons(i); // Clear the button states
      }
      for (uint8_t i = 0; i < NUM_PADS; i++) {
        //keyEvent = pads[i].pollEvents();
        //auto events = pads[i]->pollEvents();
        use_qwstpad = padLogic[i].use_qwstpad;
        if (use_qwstpad) {
          // Note: getButtonBitfield() calls read_buttons, which calls update().
          // Poll pads[i]. Param: false = not use the fancy printing
          padLogic[i].buttons = pads[i]->getButtonBitfield(true);
          //padLogic[i].key = pads[i]->getFirstPressedButtonName();
          handleButtonPress(padLogic[i], i); // Handle button press
          
          if (padLogic[i].buttons != padLogic[i].buttons_old) {
            btn_change_cnt++;
            padLogic[i].buttons_old = padLogic[i].buttons;
          }
          delay(1); // prevent WDT 
        }
        delay(1); // More responsive  
      }
      //if (btn_change_cnt >= 1)
      //  Serial.println(F("---------------------"));
    }
    Serial.flush(); // at the end of the last Serial.println()
    delay(100); // prevent WDT
  }
}