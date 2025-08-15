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
  * Update 2025-08-15:
  * Because of electrical problems using four external I2C devices on one I2C bus,
  * I split the I2C devices two-by-two onto two I2C buses. Now all four work OK.
  * The external BME280 sensor is used by default. If you don't want/can use the BME280 #undef USE_BME.
*/
#include <Arduino.h>
#include "QwstPad.h"
#include <Wire.h>
#include <string>
#include <Adafruit_TestBed.h>
#include "Adafruit_MAX1704X.h"
#include <vector>


#define SEALEVELPRESSURE_HPA (1013.25)
#define ELEVATION_CORRECTION (25) 

#ifndef USE_BME
#define USE_BME
#endif

/*
#ifdef USE_BME
#undef USE_BME
#endif
*/

#ifdef USE_BME
#include <Adafruit_BME280.h>
Adafruit_BME280 bme;
float temperature, humidity, pressure, altitude;
#endif

Adafruit_MAX17048 max_bat;
extern Adafruit_TestBed TB; 

uint8_t NUM_PADS = 0;

struct padBtn {
  uint8_t padID = 0; // Unique identifier for the pad
  bool use_qwstpad = false;
  uint16_t address = 0;
  uint32_t buttons = 0;
  std::string key = "";
  int8_t btn_idx = -1;
  uint16_t buttons_old = 0;
  int8_t logic = -1; // set to -1 if not set
  //bool a_button_has_been_pressed = false;
  bool buttonPressed = false;
  bool lastButtonState = false;
  bool currentButtonState = false;
  unsigned long lastDebounceTime = 0;
  //bool buttonPressed[NUM_BUTTONS] = {false};
  //bool lastButtonState[NUM_BUTTONS] = {false};
  //bool currentButtonState[NUM_BUTTONS] = {false};
  //unsigned long lastDebounceTime[NUM_BUTTONS] = {0};
};

const unsigned long debounceDelay = 500; // ms

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

#define DEFAULT_I2C_PORT &Wire  // TwoWire *theWire = &Wire;    // See Adafruit_Testbed.h -- The I2C port used in scanning
#define SECONDARY_I2C_PORT &Wire1
//TwoWire I2C_1 = TwoWire(1); // Use second I2C controller  -- suggested by MS Copilot

#ifndef SDA1
#define SDA1 10
#endif

#ifndef SCL1
#define SCL1 11 
#endif

#define CURRENT_MAX_PADS 2

// Create a QwstPad instance
// QwstPad* pad;  // Global pointer
QwstPad* pads[CURRENT_MAX_PADS];  // Declare globally as pointers

padBtn padLogic[CURRENT_MAX_PADS]; // Logic array aligned with pads

void blink_a_led(padBtn &padLogic, bool all_leds = false) {
  static constexpr const char txt0[] PROGMEM = "blink_a_led(): ";
  uint8_t pad_idx = padLogic.padID;

#ifdef USE_CURRENT_STATES
#undef USE_CURRENT_STATES
#endif

#ifdef USE_CURRENT_STATES
  // Iterate over all current button states
  const auto& currentStates = pads[pad_idx]->getCurrentStates();

#ifdef MY_DEBUG
  Serial.print(txt0);
  for (const auto& [key, state] : currentStates) {
    Serial.print(key.c_str());
    Serial.print(": ");
    Serial.print(state);
    Serial.print(" | ");
  }
  Serial.println();
#endif
#else
  uint8_t state = padLogic.currentButtonState;
  std::string key = padLogic.key;
#endif

#ifdef USE_CURRENT_STATES
  for (const auto& [key, state] : currentStates) {
#endif
    std::string key_mod = keyAliases.count(key) ? keyAliases[key] : key;

    if (key.empty()) {
      Serial.print(txt0);
      Serial.print(F("no key was pressed. Returning..."));
      return;
    } else {
      Serial.print(txt0);
      pads[pad_idx]->pr_PadID(); // Print pad ID

      uint8_t led_index = 0;

      // Map button name to LED index
      if (key_mod == "UP")        led_index = 1;
      else if (key_mod == "LEFT") led_index = 2;
      else if (key_mod == "RIGHT")led_index = 3;
      else if (key_mod == "DOWN") led_index = 4;
      else if (key_mod == "MINUS")led_index = 1;
      else if (key_mod == "PLUS") led_index = 4;
      else if (key_mod == "X")    led_index = 1;
      else if (key_mod == "Y")    led_index = 2;
      else if (key_mod == "A")    led_index = 3;
      else if (key_mod == "B")    led_index = 4;
      else {
        Serial.print(F("Unknown button: "));
        Serial.println(key_mod.c_str());
        return;
        //continue;
      }

#ifndef MY_DEBUG
      Serial.print(F(", LED index: "));
      Serial.print(led_index, DEC);
#endif

      Serial.print(F(", blinking one LED for button: '"));
      Serial.print(key_mod.c_str());
      Serial.println("'");

      // Blink logic
      pads[pad_idx]->clear_leds();
      delay(500);
      pads[pad_idx]->set_led(led_index, true);
      delay(500);
      pads[pad_idx]->clear_leds();
    }
#ifdef USE_CURRENT_STATES
  }
#endif

  // If no button was pressed and all_leds is true, blink all
  if (all_leds) {
    Serial.println(F("blinking all LEDs"));
    pads[pad_idx]->clear_leds();
    delay(500);
    pads[pad_idx]->set_leds(pads[pad_idx]->address_code());
    delay(500);
    pads[pad_idx]->clear_leds();
  }
}

bool ckForButtonPress() {
  static constexpr const char txt0[] PROGMEM = "ckForButtonPress(): ";
  bool retval = false;
  bool btnChanged = false;
  std::string key;
  std::string key2;
  std::string key_mod;
  uint8_t no_qwstpad = 0;
  for (uint8_t i = 0; i < CURRENT_MAX_PADS; i++) {
    bool use_qwstpad = padLogic[i].use_qwstpad;
    if (!use_qwstpad) {
      no_qwstpad++;
      if (no_qwstpad >= CURRENT_MAX_PADS) {
        Serial.print(txt0);
        pads[i]->pr_PadID();
        Serial.print(F(", no qwstpad available"));
        return retval;
      }
      continue;
    }
#ifdef MY_DEBUG
    Serial.print(txt0);
    pads[i]->pr_PadID();
    Serial.print(F(", address = 0x"));
    Serial.print(pads[i]->getAddress(), HEX);
    Serial.print(F(", use_qwstpad = "));
    Serial.println(use_qwstpad ? "true" : "false");
#endif

#ifdef MY_DEBUG
    Serial.print(txt0);
    pads[i]->pr_PadID();
    Serial.println(F(", calling update()"));
#endif
    pads[i]->update();  // needed for pollEvents

#ifdef READ_FROM_BUTTONS2
#undef READ_FROM_BUTTONS2
#endif

#ifdef READ_FROM_BUTTONS2
    std::map<std::string, bool> button_states = pads[i]->read_buttons2();
#endif 

#ifdef MY_DEBUG
    Serial.println(F("Testing read_buttons2(), Pad: "));
    pads[i]->pr_PadID();
#endif

#ifdef READ_FROM_BUTTONS2
    for (const auto& [key2, value] : button_states) {
      if (value > 0) {
        Serial.print(txt0);
        pads[i]->pr_PadID();
        Serial.print(F(", button_states from ->read_buttons2(), key_mod: \""));
        key_mod = keyAliases.count(key2) ? keyAliases[key2] : key2;
        Serial.print(key_mod.c_str());
        Serial.print(F("\", value: "));
        Serial.println(value);
      }
    }
#endif
    // Read button states if you want to debug print them
    // bool btnPressed = pads[i]->debugPrintStates();
#ifdef MY_DEBUG
    Serial.print(txt0);
    pads[i]->pr_PadID();
    Serial.println(F(", polling this pad"));
#endif
    std::vector<ButtonEvent> keyEvent = pads[i]->pollEvents();
    size_t EventSz = keyEvent.size();
#ifdef MY_DEBUG
    printPadDiagnostics(i);
#endif
    //while (Serial.available() == 0) {
      // Wait for user input
    //}
    //Serial.read(); // Clear the input

#ifdef MY_DEBUG
    Serial.print(txt0);
    pads[i]->pr_PadID();
    Serial.print(F(", EventSz = "));
    Serial.println(EventSz);
#endif
    if (EventSz > 0) {
      padLogic[i].buttons_old = padLogic[i].buttons; // save current buttens state
      // Serial.print("Event count: ");
      // Serial.println(EventSz);
      for (const ButtonEvent& event : keyEvent) {
        key = event.key;
        key_mod = keyAliases.count(key) ? keyAliases[key] : key;
        btnChanged = pads[i]->buttonChanged(key);
#ifndef MY_DEBUG
        Serial.print(txt0);
        pads[i]->pr_PadID();
        Serial.print(F(", btnChanged = "));
        Serial.println(btnChanged ? "true" : "false");
#endif
        if (btnChanged) {
#ifndef MY_DEBUG
          Serial.print(txt0);
          pads[i]->pr_PadID();
          Serial.print(F(", button state changed for: '"));
          Serial.print(key_mod.c_str());
          Serial.print(F("', button: "));
          Serial.println(event.type == PRESSED ? "PRESSED" : "RELEASED");
#endif
          if (event.type == PRESSED) {
            unsigned long currentTime = millis();
            int8_t btn_idx = pads[i]->getFirstPressedButtonBitIndex();
#ifndef MY_DEBUG
            Serial.print(txt0);
            pads[i]->pr_PadID();
            Serial.print(F(", btn_idx = "));
            Serial.println(btn_idx);
#endif
            if (btn_idx != -1) {
              if ((currentTime - padLogic[i].lastDebounceTime) > debounceDelay) { // && padLogic.currentButtonState[btn_idx]) {
                // Only handle out of debounceDelay
                retval = true;
                padLogic[i].key = key;
                padLogic[i].btn_idx = btn_idx;
                padLogic[i].buttons = pads[i]->getButtonBitfield(false, false); // do not buttons, example: 01000 00000 00000
                padLogic[i].buttonPressed = true;
                padLogic[i].lastDebounceTime = currentTime;
                padLogic[i].lastButtonState = padLogic[i].currentButtonState; // store the "old" currentButtonState
                padLogic[i].currentButtonState = true; // set the currentButtonState
#ifndef MY_DEBUG
                Serial.print(txt0);
                Serial.println(F("going to call blink_a_led() ..."));
                blink_a_led(padLogic[i], false); // Blink an individual LED
#endif
#ifndef MY_DEBUG
                Serial.print(F("ckForButtonPress(): Button: "));
                Serial.print(btn_idx);
                Serial.print(F(" = \'"));
                Serial.print(key_mod.c_str());
                Serial.print(F("\', pressed at time: "));
                Serial.println(currentTime);
#endif
                return retval;
              }
            }
            else
              padLogic[i].btn_idx = btn_idx; // set to -1
          }
        }
      }
      //if (btnPressed)
      //  delay(3000);  // Give user time to view the results
    }
  }
  return retval;
}

void clr_pad_stru(uint8_t PadNr) {
  padBtn empty; // Default-initialized struct
  if (PadNr >= 0 && PadNr < CURRENT_MAX_PADS) {
    padLogic[PadNr] = empty;
    padLogic[PadNr].padID = PadNr; // Restore ID
    if (PadNr == 0)
      padLogic[PadNr].address = DEFAULT_ADDRESS;
    else if (PadNr == 1)
      padLogic[PadNr].address = ALT_ADDRESS_1;
    else if (PadNr == 2)
      padLogic[PadNr].address = ALT_ADDRESS_2;
    else if (PadNr == 3)
      padLogic[PadNr].address = ALT_ADDRESS_3;
    if (pads[PadNr]->isConnected())
      padLogic[PadNr].use_qwstpad = true;
    else 
      padLogic[PadNr].use_qwstpad = false;
    padLogic[PadNr].key = "";
    padLogic[PadNr].btn_idx = -1;
    padLogic[PadNr].logic = pads[PadNr]->getLogicType();
  }
}

void show_pad_stru(uint8_t PadNr) {
  if (PadNr >= 0 && PadNr < CURRENT_MAX_PADS) {
    Serial.print(F("padID:              "));
    Serial.println(padLogic[PadNr].padID); // Unique identifier for the pad
    Serial.print(F("use_qwstpad:        "));
    Serial.print(padLogic[PadNr].use_qwstpad);
    Serial.print(F(" = "));
    Serial.println(padLogic[PadNr].use_qwstpad ? "true" : "false");
    Serial.print(F("address:            0x"));
    Serial.println(padLogic[PadNr].address,HEX);
    Serial.print(F("buttons:            "));
    Serial.println(padLogic[PadNr].buttons,BIN);
    Serial.print(F("key:                \""));
    Serial.print(padLogic[PadNr].key.c_str());
    Serial.println(F("\""));
    Serial.print(F("btn_idx:            "));
    Serial.println(padLogic[PadNr].btn_idx);
    Serial.print(F("buttons_old:        "));
    Serial.println(padLogic[PadNr].buttons_old);
    Serial.print(F("logic:              "));
    Serial.print(padLogic[PadNr].logic);
    Serial.print(F(" = "));
    if (padLogic[PadNr].logic == 0)
      Serial.println(F("ACTIVE_HIGH"));
    else if (padLogic[PadNr].logic == 1)
      Serial.println(F("ACTIVE_LOW"));
    else
      Serial.println(F("UNKNOWN"));
    Serial.print(F("buttonPressed:      "));
    Serial.println(padLogic[PadNr].buttonPressed);
    Serial.print(F("lastButtonState:    "));
    Serial.println(padLogic[PadNr].lastButtonState);
    Serial.print(F("currentButtonState: "));
    Serial.println(padLogic[PadNr].currentButtonState);
    Serial.print(F("lastDebounceTime:   "));
    Serial.println(padLogic[PadNr].lastDebounceTime);
  }
}

const std::vector<int> BUTTON_PINS = {1, 2, 3, 4, 5, 11, 12, 13, 14, 15};
const std::vector<int> LED_PINS = {6, 7, 9, 10};
// Pin 8 is excluded

void printPadDiagnostics(uint8_t PadNr) {
    bool btnPressed = false;
    Serial.println("🔘 Button States:");
    for (int pin : BUTTON_PINS) {
        bool state = pads[PadNr]->readRawPin(pin);
        Serial.print("  Pin ");
        Serial.print(pin);
        Serial.print(": ");
        Serial.println(state ? "HIGH (pressed)" : "LOW (released)");
        if (state)
          btnPressed = true;
        //while (Serial.available() == 0) {
          // Wait for user input
        //}
        //Serial.read(); // Clear the input
    }

    Serial.println("💡 LED States:");
    for (int pin : LED_PINS) {
        bool state = pads[PadNr]->readRawPin(pin);
        Serial.print("  Pin ");
        Serial.print(pin);
        Serial.print(": ");
        Serial.println(state ? "ON" : "OFF");
    }
    if (btnPressed)
      delay(3000);
}

#ifdef USE_BME
void read_bme280_data()
{
  static constexpr const char txt0[] PROGMEM = "read_bme280_data(): ";
  // Read temperature, pressure, altitude and humidity
  temperature = bme.readTemperature();
  pressure = bme.readPressure() / 100.0F; // Convert Pa to mBar
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA) + ELEVATION_CORRECTION; // Altitude in meters
  humidity = bme.readHumidity();

#ifndef MY_DEBUG
  Serial.print(txt0);
  Serial.print(F("Temp: "));
  Serial.print(temperature);
  Serial.print(F(" °C, Pressure: "));
  Serial.print(pressure);
  Serial.print(F(" mBar, Altitude: "));
  Serial.print(altitude);
  Serial.print(F(" m, Humidity: "));
  Serial.print(humidity);
  Serial.println(F(" %"));
#endif
}
#endif

void clr_buttons(uint8_t i, bool all = false) {
  if (i < 0 || i >= CURRENT_MAX_PADS) {
    Serial.println(F("Invalid pad index in clr_buttons()"));
    return;
  }

  if (all) { // Clear all pads
    for (int i = 0; i < CURRENT_MAX_PADS; ++i) {
      if (padLogic[i].use_qwstpad) {
        padLogic[i].key = ""; // Clear the key
        padLogic[i].btn_idx = -1;
        padLogic[i].buttons = 0; // Clear the button state
        padLogic[i].buttons_old = 0; // Clear the old button state
        padLogic[i].buttonPressed = 0; // Clear the button state
        padLogic[i].lastButtonState = false; // Reset last button state
        padLogic[i].currentButtonState = false; // Reset current button state
        padLogic[i].lastDebounceTime = 0; // Reset debounce time
      }

    }
  } else { // Clear only the specified pad
    if (padLogic[i].use_qwstpad) {
      padLogic[i].key = ""; // Clear the key
      padLogic[i].btn_idx = -1; // Clear the btn_idx
      padLogic[i].buttons = 0; // Clear the button state
      padLogic[i].buttons_old = 0; // Clear the old button state
      padLogic[i].buttonPressed = 0; // Clear the button state
      padLogic[i].lastButtonState = false; // Reset last button state
      padLogic[i].currentButtonState = false; // Reset current button state
      padLogic[i].lastDebounceTime = 0; // Reset debounce time
    }
  }
}

void setup() {
  static constexpr const char txt0[] PROGMEM = "setup(): ";
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("QwstPad ButtonEvent 2 QwstPad test"));

  // turn on the TFT / I2C power supply
#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_TFT)
  Serial.println("✅ Adafruit Feather ESP32-S3 TFT detected");
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
#else
  Serial.println("❌ Board macro not defined");
#endif


  if (!max_bat.begin()) {
      Serial.println(F("❌ Couldn\'t find Adafruit MAX1704X?\nMake sure a battery is plugged in!"));
      while (1) delay(10);
  } else {
    Serial.print(F("✅ Found MAX17048"));
    Serial.print(F(" with Chip ID: 0x")); 
    Serial.println(max_bat.getChipID(), HEX);
  }

#if defined(DEFAULT_I2C_PORT)
  Serial.print("Default I2C port (Wire) ");
  TB.theWire = DEFAULT_I2C_PORT;
  TB.printI2CBusScan();
#endif
  
  Wire.begin();

  Wire.setClock(100000); // Optional: slow down I2C


#if defined(SECONDARY_I2C_PORT)
  //Wire1.begin(SDA1, SCL1, 100000);
  Serial.print("Secondary I2C port (Wire1) ");
  TB.theWire = SECONDARY_I2C_PORT;
  TB.theWire->setPins(SDA1, SCL1);
  TB.printI2CBusScan();
  TB.theWire->begin();
#endif

#ifdef USE_BME
  Serial.println("Initializing BME280...");
  if (!bme.begin(0x76)) {
    Serial.println("BME280 not found or failed to initialize.");
  } else {
    Serial.println("BME280 initialized successfully.");
  }
#endif

  pads[0] = new QwstPad(SECONDARY_I2C_PORT, DEFAULT_ADDRESS, true);
  pads[1] = new QwstPad(SECONDARY_I2C_PORT, ALT_ADDRESS_1, true);
  //pads[0] = new QwstPad(DEFAULT_I2C_PORT, 0x21, true); // DEFAULT_ADDRESS);
  //pads[1] = new QwstPad(DEFAULT_I2C_PORT, 0x23, true); // ALT_ADDRESS_1);
  //pads[2] = new QwstPad(DEFAULT_I2C_PORT, ALT_ADDRESS_2, true);
  //pads[3] = new QwstPad(DEFAULT_I2C_PORT, ALT_ADDRESS_3, true);

  uint8_t pad1address;
  Serial.print(F("Maximum number of QwSTPads: "));
  Serial.println(CURRENT_MAX_PADS);
  for (int i = 0; i < CURRENT_MAX_PADS; ++i) {
    //pads[i]->begin();
    bool isConnected = pads[i]->isConnected();
    uint8_t pAddress = pads[i]->getAddress();
    pads[i]->pr_PadID();
    Serial.print(F(", I2C address: 0x"));
    Serial.print(pads[i]->getAddress(), HEX);
    Serial.print(F(", padID: "));
    padLogic[i].padID = pads[i]->getpadIDFromAddress(pads[i]->getAddress()); //+1; // adjust for human readable 1...4
    Serial.print(padLogic[i].padID);
    if (isConnected) {  
      if (i == 0) {
        pad1address == pAddress;
        Serial.println(F(". Pad is connected"));
        padLogic[i].use_qwstpad = true;
        padLogic[i].logic = pads[i]->getLogicType();
        NUM_PADS++;
      }
      else if (i > 0) {
        // It happened that a pad with padID > 0 had the same address as the one with padID 0.
        // so we introduced pad1address to compare
        if (pad1address != pAddress) {
          Serial.println(F(". Pad is connected"));
          padLogic[i].address = pAddress;
          padLogic[i].use_qwstpad = true;
          padLogic[i].logic = pads[i]->getLogicType();
          NUM_PADS++;
        }
        else {
          padLogic[i].address = 0;
          padLogic[i].use_qwstpad = false;
          padLogic[i].logic = -1;
        }
      }
    }
    else {
      Serial.println(F(". Pad is not connected"));
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
      Serial.print(txt0);
      pads[i]->pr_PadID();
      Serial.print(F(", initialized with address: 0x"));
      Serial.println(pads[j]->getAddress(), HEX);
#endif
    }
  }
  //delay(1000); // advised by MS Copilot
  for (uint8_t i = 0; i < NUM_PADS; i++) {
      Serial.print(txt0);
      pads[i]->pr_PadID();
      Serial.print(F(", use_qwstpad = "));
      Serial.println(padLogic[i].use_qwstpad ? "true" : "false");
  }
  pads[0]->printAllPadConfigs(); // Any pad instance can call it

  //delay(10000); // wait 10 seconds
}

bool start = true;

void loop() {
  static constexpr const char txt0[] PROGMEM = "loop(): ";
  bool  use_qwstpad;
  uint16_t padLogicLen = 0;
  static unsigned long lastPollTime = 0;
  const unsigned long pollInterval = 50;
  static unsigned long lastTpaTime = 0;
  const unsigned long tpaInterval = 1 * 60 * 1000; // 1 minute temperatur, pressure and humidity measure interval
  unsigned long currentTime = millis();

  if (start || currentTime - lastTpaTime >= tpaInterval) {
    start = false;
    lastTpaTime = currentTime;
  #ifdef USE_BME
    read_bme280_data();
  #endif
  }

  if (currentTime - lastPollTime >= pollInterval) {
    lastPollTime = currentTime;

    for (uint8_t i = 0; i < NUM_PADS; i++) {
      pads[i]->clear_leds(); // Clear all LEDs
      clr_buttons(i); // Clear the button states for pad i
    }

    if (ckForButtonPress()) {
      Serial.print(txt0);
      Serial.println(F("a button has been pressed"));
    }
/*
    for (uint8_t i = 0; i < NUM_PADS; i++) {
      if (padLogic[i].use_qwstpad) {

        pads[i]->update();

        std::vector<ButtonEvent> keyEvent = pads[i]->pollEvents();
        
        size_t EventSz = keyEvent.size();
 
        if (EventSz > 0) {
#ifndef MY_DEBUG
          Serial.print(txt0);
          pads[i]->pr_PadID();
          Serial.print(F(", event count: "));
          Serial.println(EventSz);
#endif
          for (const ButtonEvent& event : keyEvent) {
            std::string key = event.key;
            std::string key_mod = keyAliases.count(key) ? keyAliases[key] : key;
            if (pads[i]->buttonChanged(key)) {
              pads[i]->pr_PadID();
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
*/
  }
}
