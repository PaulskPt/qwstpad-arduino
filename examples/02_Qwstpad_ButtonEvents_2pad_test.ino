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

#define DEFAULT_I2C_PORT &Wire
#define CURRENT_MAX_PADS 2

// Create a QwstPad instance
// QwstPad* pad;  // Global pointer
QwstPad* pads[CURRENT_MAX_PADS];  // Declare globally as pointers

padBtn padLogic[CURRENT_MAX_PADS]; // Logic array aligned with pads

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
/*
  pad = new QwstPad(DEFAULT_I2C_PORT, DEFAULT_ADDRESS);  // Instantiate with required args
  pad->begin();  // Initialize the pad (assuming this sets up pins, etc.)
  pad->setLogicType(ACTIVE_HIGH);  // Configure logic type via method
  bool isConnected = pad->isConnected();
  uint16_t pAddress = pad->getAddress();
  static constexpr const char* txts[] PROGMEM = {"not ", "connected"};
  Serial.print(F("Pad is "));
  if (isConnected) {  
    padLogic[0].address = pAddress;
    padLogic[0].use_qwstpad = true;
	  Serial.println(txts[1]);
    Serial.print(F("Address: 0x"));
    Serial.println(pAddress, HEX);
  	NUM_PADS++;
  } else {
    Serial.print(txts[0]);
    Serial.println(txts[1]);
    padLogic[0].address = 0;
    padLogic[0].use_qwstpad = false;
  }
*/
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

  if (currentTime - lastPollTime >= pollInterval) {
    lastPollTime = currentTime;

    for (uint8_t i = 0; i < NUM_PADS; i++) {
      //keyEvent = pads[i].pollEvents();
      //auto events = pads[i]->pollEvents();
      use_qwstpad = padLogic[i].use_qwstpad;
      if (use_qwstpad) {
        pads[i]->update();

        // Read button states if you want to debug print them
        // bool btnPressed = pad->debugPrintStates();

        std::vector<ButtonEvent> keyEvent = pads[i]->pollEvents();

        size_t EventSz = keyEvent.size();
        std::string key;
        std::string key_mod = keyAliases.count(key) ? keyAliases[key] : key;
        
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
            }
          }
        }
        //if (btnPressed)
        //  delay(3000);  // Give user time to view the results
      }
    }
  }
}
