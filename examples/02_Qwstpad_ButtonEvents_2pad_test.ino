/* 
  * 02_Qwstpad_ButtonEvents_2pad_test.ino
  *
  * Example of howto work with Button Events for two QwstPads.
  * Created by Paulus Schulinck (Github handle: @PaulskPt),
  * with assistance of Microsoft Copilot.
  * Date: 2025-08-11
  * License: MIT License]
  * 
  * This assumes pollEvents() returns something iterable like std::vector<ButtonEvent> or similar.
  * If ButtonEventType is a scoped enum (enum class), you’ll need to qualify it like ButtonEventType::Pressed.
  * You can expand this to trigger actions — e.g., toggling LEDs, sending MIDI messages, etc.
  *
  * The QwstPad library is used to interface with the Pimoroni Qw/ST Pad (I2C Game Controller) board.
  * Product info: https://shop.pimoroni.com/products/qwst-pad?variant=53514400596347
  * This library is ported by me from the Pimoroni qwstpad-micropython library.
  * See: https://github.com/pimoroni/qwstpad-micropython/blob/main/src/qwstpad.py
  * This sketch uses the C++17 language standard version.
*/
#include <Arduino.h>
#include "QwstPad.h"
#include <Wire.h>
#include <string>
#include <Adafruit_TestBed.h>
#include "Adafruit_MAX1704X.h"
#include <vector>

Adafruit_MAX17048 max_bat;
extern Adafruit_TestBed TB;

uint8_t NUM_PADS = 0;

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

std::map<std::string, std::string> keyAliases = {
  {"X", "X"},
  {"Y", "Y"},
  {"A", "A"},
  {"B", "B"},  
  {"P", "PLUS"},
  {"M", "MINUS"},
  {"U", "UP"},
  {"L", "LEFT"},
  {"R", "RIGHT"},
  {"D", "DOWN"}
};

#define DEFAULT_I2C_PORT &Wire
#define CURRENT_MAX_PADS 2

// Create a QwstPad instance
// QwstPad* pad;  // Global pointer
QwstPad* pads[CURRENT_MAX_PADS];  // Declare globally as pointers

padBtn padLogic[CURRENT_MAX_PADS]; // Logic array aligned with pads

void blink_a_led(padBtn &padLogic, bool all_leds = false) {
  static constexpr const char txt0[] PROGMEM = "blink_a_led(): ";
  uint8_t pad_idx = padLogic.padID;
  int8_t btn_idx = pads[pad_idx]->getFirstPressedButtonBitIndex();
  std::string key = pads[pad_idx]->getFirstPressedButtonName();
  std::string key_mod = keyAliases.count(key) ? keyAliases[key] : key;
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
#ifdef MY_DEBUG
      Serial.print(F(", LED index: "));
      Serial.println(led_index, DEC);
#endif
      Serial.print(F(", blinking one LED for button: \'"));
      //Serial.println(btn_idx);
      Serial.print(key_mod.c_str());
      Serial.println("\'");
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

  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("QwstPad ButtonEvent test"));

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
  static unsigned long lastPollTime = 0;
  const unsigned long pollInterval = 50;
  unsigned long currentTime = millis();

  if (currentTime - lastPollTime >= pollInterval) {
    lastPollTime = currentTime;

    for (uint8_t i = 0; i < NUM_PADS; i++) {
      pads[i]->clear_leds(); // Clear all LEDs
      clr_buttons(i); // Clear the button states
    }

    for (uint8_t i = 0; i < NUM_PADS; i++) {
      //keyEvent = pads[i].pollEvents();
      //auto events = pads[i]->pollEvents();
      use_qwstpad = padLogic[i].use_qwstpad;
      if (use_qwstpad) {
        pads[i]->update();

        // Read button states if you want to debug print them
        // bool btnPressed = pads[i]->debugPrintStates();

        std::vector<ButtonEvent> keyEvent = pads[i]->pollEvents();

        size_t EventSz = keyEvent.size();
        //std::string key;
        //std::string key_mod = keyAliases.count(key) ? keyAliases[key] : key;

        if (EventSz > 0) {
          // Serial.print("Event count: ");
          // Serial.println(EventSz);
          for (const ButtonEvent& event : keyEvent) {
            std::string key = event.key;
            std::string key_mod = keyAliases.count(key) ? keyAliases[key] : key;
            if (pads[i]->buttonChanged(key)) {
              Serial.print("Pad ");
              Serial.print(i + 1);
              Serial.print(", button state changed for: '");
              Serial.print(key_mod.c_str());
              Serial.print("', button: ");
              Serial.println(event.type == PRESSED ? "PRESSED" : "RELEASED");
              if (event.type == PRESSED) {
                 blink_a_led(padLogic[i], false); // Blink an individual LED
              }
            }
          }
        }
        //if (btnPressed)
        //  delay(3000);  // Give user time to view the results
      }
    }
  }
}
