#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>           // Required for hardware channel locking
#include <esp_arduino_version.h> // Explicitly pull ESP32 Core version macros
#include <U8g2lib.h>
#include <Wire.h>

// --- Conditional Chip/Pin Mappings (ESP32-C6 vs ESP32-C3 Backward Compatibility) ---
#if defined(CONFIG_IDF_TARGET_ESP32C6)
  #define BTN_CONFIRM     0
  #define BTN_PSH         1
  #define ENCODER_TRA     2
  #define ENCODER_TRS     21
  #define OLED_SDA        22
  #define OLED_SCL        23
  #define BTN_BACK        16
#else
  #define ENCODER_TRA     0
  #define ENCODER_TRS     1
  #define BTN_PSH         2
  #define BTN_CONFIRM     3
  #define BTN_BACK        10
  #define OLED_SDA        8
  #define OLED_SCL        9
#endif

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

const int ROWS = 4;
const int COLS = 3;
const int NAV_ITEMS = 10; // Nine normal cells plus one full-width settings row.
int cursorRow = 0;
int cursorCol = 0;
bool isEditing = false;


static const uint8_t ICON_CONNECTED[] PROGMEM = {
  0x18, 0x24, 0x42, 0x81, 0x81, 0x42, 0x24, 0x18
};

// Connected: simple Wi-Fi glyph (8x8)
static const uint8_t ICON_WIFI_CONNECTED[] PROGMEM = {
  0x00, 0x3C, 0x42, 0x18, 0x24, 0x00, 0x18, 0x00
};

// Disconnected: X glyph (8x8)
static const uint8_t ICON_DISCONNECTED_X[] PROGMEM = {
  0x81, 0x42, 0x24, 0x18, 0x18, 0x24, 0x42, 0x81
};

// Brightness: sun glyph (8x8)
static const uint8_t ICON_SUN[] PROGMEM = {
  0x24, 0x00, 0x5A, 0x3C, 0x3C, 0x5A, 0x00, 0x24
};

// Zone icons (replace these bitmaps with your own later)
static const uint8_t ICON_ZONE_1[] PROGMEM = {
  0x01, 0x21, 0x31, 0xf9, 0xf9, 0x31, 0x21, 0x01
};
static const uint8_t ICON_ZONE_2[] PROGMEM = {
 0xff, 0x00, 0x00, 0x18, 0x3c, 0x7e, 0x18, 0x18
};
static const uint8_t ICON_ZONE_3[] PROGMEM = {
  0x80, 0x84, 0x8c, 0x9f, 0x9f, 0x8c, 0x84, 0x80
};
static const uint8_t ICON_ZONE_GLOBAL[] PROGMEM = {
  0x3c, 0x5a, 0x8d, 0x87, 0xf1, 0xaf, 0x4e, 0x3c
};

static const uint8_t* getSelectedZoneIcon(int zoneIdx) {
  switch (zoneIdx) {
    case 0: return ICON_ZONE_1;
    case 1: return ICON_ZONE_2;
    case 2: return ICON_ZONE_3;
    default: return ICON_ZONE_GLOBAL; // zoneIdx == 3
  }
}

static const uint8_t ICON_MENU[] PROGMEM = {
  0x00, 0x7E, 0x7E, 0x00, 0x7E, 0x7E, 0x00, 0x7E
};


enum LightMode { OFF, STABLE, RAINBOW, BREATHING, MUSIC };
const char* modeNames[] = {"Off", "Stable", "Rainbow", "Breath", "Music"};
enum ColorChoice {
  COLOR_RED,
  COLOR_GREEN,
  COLOR_BLUE,
  COLOR_PURPLE,
  COLOR_WHITE,
  COLOR_WARM,
  COLOR_CYAN,
  COLOR_YELLOW,
  COLOR_ORANGE,
  COLOR_PINK,
  COLOR_TEAL,
  COLOR_CUSTOM
};
const char* colorNames[] = {
  "Red", "Green", "Blue", "Purple", "White", "Warm",
  "Cyan", "Yellow", "Orange", "Pink", "Teal", "Custom"
};

struct ZoneSettings {
  uint8_t r = 180;
  uint8_t g = 50;
  uint8_t b = 250;
  uint8_t customR = 180;
  uint8_t customG = 50;
  uint8_t customB = 250;
  ColorChoice colorChoice = COLOR_CUSTOM;
  uint8_t brightness = 150;
  LightMode mode = STABLE;
  uint16_t lightCount = 30;
  uint8_t scatter = 0;
  uint8_t rainbowSpeed = 20;
  uint8_t musicSensitivity = 128;
};

ZoneSettings zones[4];      // 0, 1, 2 = Physical Zones | 3 = Global Control
int currentZoneIdx = 3;
bool inCustomColorSubmenu = false;
bool inSettingsMenu = false;
uint8_t settingsCursor = 0;
bool automaticLights = false;
bool useFahrenheit = true;
float temperatureOffsetC = 0.0f;

#define TELEMETRY_MAGIC 0xA17C
#define TELEMETRY_RADAR_CONNECTED 0x01
#define TELEMETRY_AHT_CONNECTED   0x02
#define TELEMETRY_PRESENCE        0x04

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct __attribute__((packed)) ESPNowPacket {
  uint8_t targetZone;
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t brightness;
  uint8_t mode;
  uint16_t lightCount;
  uint8_t scatter;
  uint8_t rainbowSpeed;
  uint8_t musicSensitivity;
  uint8_t automaticLights;
};

struct __attribute__((packed)) TelemetryPacket {
  uint16_t magic;
  int16_t temperatureCentiC;
  uint16_t humidityCentiPct;
  uint8_t flags;
};

volatile int encoderDelta = 0;
volatile unsigned long lastHeartbeatReceived = 0;
volatile int16_t receivedTemperatureCentiC = 0;
volatile uint16_t receivedHumidityCentiPct = 0;
volatile uint8_t receivedTelemetryFlags = 0;
volatile bool displayDirty = true;

unsigned long lastUserActivity = 0;
unsigned long lastDisplayRefresh = 0;
bool lastReceiverConnected = false;

const uint32_t CONFIRM_LONG_PRESS_MS = 700;
const uint32_t DEBOUNCE_MS = 35;
const uint32_t BACK_DEBOUNCE_MS = 150;
const uint32_t DISPLAY_FRAME_INTERVAL_MS = 67;       // At most ~15 FPS.
const uint32_t AUTO_SLEEP_TIMEOUT_MS = 5UL * 60UL * 1000UL;

void handleEncoder();
void applyEncoderStep(int direction);
void handleButtons();
void updateDisplay();
void broadcastSettings(int zoneIdx);
void IRAM_ATTR handleEncoderISR();
void enterSleepMode();
void initWireless();
void waitForButtonRelease(int pin);
void syncGlobalToPhysical();
void toggleSpecificZone(int zoneIdx);
void toggleGlobalZones();
void applyColorChoice(ZoneSettings &z);
void toggleOffStableMode(ZoneSettings &z);
void noteUserActivity();

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status);
void OnDataRecv(const esp_now_recv_info_t * recv_info, const uint8_t *incomingData, int len);
#else
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
#endif

void syncGlobalToPhysical() {
  for (int i = 0; i < 3; i++) {
    zones[i] = zones[3];
  }
}

void toggleSpecificZone(int zoneIdx) {
  if (zoneIdx < 0 || zoneIdx > 2) return;
  toggleOffStableMode(zones[zoneIdx]);
}

void toggleGlobalZones() {
  LightMode prevMode = zones[3].mode;
  toggleOffStableMode(zones[3]);

  if (zones[3].mode == STABLE && prevMode == OFF) {
    syncGlobalToPhysical();
  } else if (zones[3].mode == OFF && prevMode != OFF) {
    for (int i = 0; i < 3; i++) {
      zones[i].mode = OFF;
    }
  }
}

void toggleOffStableMode(ZoneSettings &z) {
  if (z.mode == OFF) z.mode = STABLE;
  else z.mode = OFF;
}

void applyColorChoice(ZoneSettings &z) {
  switch (z.colorChoice) {
    case COLOR_RED:
      z.r = 255; z.g = 0; z.b = 0;
      break;
    case COLOR_GREEN:
      z.r = 0; z.g = 255; z.b = 0;
      break;
    case COLOR_BLUE:
      z.r = 0; z.g = 0; z.b = 255;
      break;
    case COLOR_PURPLE:
      z.r = 180; z.g = 50; z.b = 250;
      break;
    case COLOR_WHITE:
      z.r = 255; z.g = 255; z.b = 255;
      break;
    case COLOR_WARM:
      z.r = 255; z.g = 147; z.b = 41;
      break;
    case COLOR_CYAN:
      z.r = 0; z.g = 255; z.b = 255;
      break;
    case COLOR_YELLOW:
      z.r = 255; z.g = 255; z.b = 0;
      break;
    case COLOR_ORANGE:
      z.r = 255; z.g = 120; z.b = 0;
      break;
    case COLOR_PINK:
      z.r = 255; z.g = 40; z.b = 120;
      break;
    case COLOR_TEAL:
      z.r = 0; z.g = 160; z.b = 130;
      break;
    case COLOR_CUSTOM:
    default:
      z.r = z.customR; z.g = z.customG; z.b = z.customB;
      break;
  }
}

void initWireless() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW! Resetting...");
    ESP.restart();
  }

  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to register broadcast peer!");
  }
}

void waitForButtonRelease(int pin) {
  unsigned long start = millis();
  while (digitalRead(pin) == LOW && millis() - start < 1500) {
    delay(10);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Booting ARGB Wireless Controller Transmitter...");

  initWireless();

  pinMode(ENCODER_TRA, INPUT_PULLUP);
  pinMode(ENCODER_TRS, INPUT_PULLUP);
  pinMode(BTN_PSH, INPUT_PULLUP);
  pinMode(BTN_CONFIRM, INPUT_PULLUP);
  pinMode(BTN_BACK, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_TRA), handleEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_TRS), handleEncoderISR, CHANGE);

  Wire.begin(OLED_SDA, OLED_SCL);
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setPowerSave(0);
  lastUserActivity = millis();

  broadcastSettings(currentZoneIdx);
}

void loop() {
  handleEncoder();
  handleButtons();

  unsigned long now = millis();
  bool receiverConnected = now - lastHeartbeatReceived < 5000;
  if (receiverConnected != lastReceiverConnected) {
    lastReceiverConnected = receiverConnected;
    displayDirty = true;
  }

  if (now - lastUserActivity >= AUTO_SLEEP_TIMEOUT_MS) {
    enterSleepMode();
    now = millis();
  }

  if (displayDirty && now - lastDisplayRefresh >= DISPLAY_FRAME_INTERVAL_MS) {
    displayDirty = false;
    lastDisplayRefresh = now;
    updateDisplay();
  }
  delay(5);
}

void noteUserActivity() {
  lastUserActivity = millis();
  displayDirty = true;
}

void IRAM_ATTR handleEncoderISR() {
  static uint8_t old_SEC = 3;
  uint8_t SEC = (digitalRead(ENCODER_TRA) << 1) | digitalRead(ENCODER_TRS);
  if (SEC != (old_SEC & 0x03)) {
    old_SEC = (old_SEC << 2) | SEC;
    old_SEC &= 0x0F;
    static const int8_t KNOBDIR[] = {
       0, -1,  1,  0,
       1,  0,  0, -1,
      -1,  0,  0,  1,
       0,  1, -1,  0
    };
    encoderDelta += KNOBDIR[old_SEC];
  }
}

void broadcastSettings(int zoneIdx) {
  ESPNowPacket packet;
  packet.targetZone = zoneIdx;
  packet.r = zones[zoneIdx].r;
  packet.g = zones[zoneIdx].g;
  packet.b = zones[zoneIdx].b;
  packet.brightness = zones[zoneIdx].brightness;
  packet.mode = (uint8_t)zones[zoneIdx].mode;
  packet.lightCount = zones[zoneIdx].lightCount;
  packet.scatter = zones[zoneIdx].scatter;
  packet.rainbowSpeed = zones[zoneIdx].rainbowSpeed;
  packet.musicSensitivity = zones[zoneIdx].musicSensitivity;
  packet.automaticLights = automaticLights ? 1 : 0;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&packet, sizeof(packet));
  if (result == ESP_OK) {
    Serial.printf("Transmitted settings for Zone %d.\n", zoneIdx);
  } else {
    Serial.println("Error transmitting settings!");
  }
}

void handleEncoder() {
  int dir = 0;
  noInterrupts();
  dir = encoderDelta;
  encoderDelta = 0;
  interrupts();

  if (dir == 0) return;

  static int accumulatedTicks = 0;
  accumulatedTicks += dir;

  int clicks = accumulatedTicks / 4;
  accumulatedTicks %= 4;

  if (clicks != 0) {
    noteUserActivity();
    int direction = (clicks > 0) ? 1 : -1;
    int absClicks = abs(clicks);
    for (int i = 0; i < absClicks; i++) {
      applyEncoderStep(direction);
    }
    // Cursor movement and local-only settings do not need an ESP-NOW packet.
    if (isEditing && !inSettingsMenu) broadcastSettings(currentZoneIdx);
  }
}

void applyEncoderStep(int direction) {
  ZoneSettings &z = zones[currentZoneIdx];

  if (inSettingsMenu) {
    if (isEditing && settingsCursor == 2) {
      float stepC = useFahrenheit ? (0.1f / 1.8f) : 0.1f;
      temperatureOffsetC = constrain(temperatureOffsetC + (direction * stepC), -10.0f, 10.0f);
    } else {
      settingsCursor = constrain((int)settingsCursor + direction, 0, 2);
    }
    return;
  }

  if (inCustomColorSubmenu) {
    if (!isEditing) {
      cursorRow = constrain(cursorRow + direction, 0, 2);
      cursorCol = 0;
    } else {
      if (cursorRow == 0) z.customR = constrain(z.customR + (direction * 8), 0, 255);
      if (cursorRow == 1) z.customG = constrain(z.customG + (direction * 8), 0, 255);
      if (cursorRow == 2) z.customB = constrain(z.customB + (direction * 8), 0, 255);
      if (z.colorChoice == COLOR_CUSTOM) {
        z.r = z.customR;
        z.g = z.customG;
        z.b = z.customB;
      }
      if (currentZoneIdx == 3) {
        syncGlobalToPhysical();
      }
    }
    return;
  }

  if (!isEditing) {
    int linearIndex = (cursorRow == 3) ? 9 : (cursorRow * COLS + cursorCol);
    linearIndex += direction;
    if (linearIndex < 0) linearIndex = NAV_ITEMS - 1;
    if (linearIndex >= NAV_ITEMS) linearIndex = 0;
    if (linearIndex == 9) {
      cursorRow = 3;
      cursorCol = 0;
    } else {
      cursorRow = linearIndex / COLS;
      cursorCol = linearIndex % COLS;
    }
  } else {
    if (cursorCol == 0) {
      if (cursorRow == 0) {
        int nextChoice = (int)z.colorChoice + direction;
        if (nextChoice < 0) nextChoice = COLOR_CUSTOM;
        if (nextChoice > COLOR_CUSTOM) nextChoice = 0;
        z.colorChoice = (ColorChoice)nextChoice;
        applyColorChoice(z);
      } else if (cursorRow == 1) {
        if (z.mode == RAINBOW) {
          z.rainbowSpeed = constrain((int)z.rainbowSpeed + (direction * 4), 0, 255);
        } else if (z.mode == MUSIC) {
          z.musicSensitivity = constrain((int)z.musicSensitivity + (direction * 4), 0, 255);
        }
      }
      if (cursorRow == 2) z.brightness = constrain(z.brightness + (direction * 8), 0, 255);
    } else if (cursorCol == 1) {
      if (cursorRow == 1) {
        int prevZone = currentZoneIdx;
        currentZoneIdx = constrain(currentZoneIdx + direction, 0, 3);

        if (currentZoneIdx != 3) {
          if (zones[currentZoneIdx].mode == OFF && zones[3].mode != OFF) {
            zones[currentZoneIdx].mode = zones[3].mode;
          }
        } else if (prevZone != 3) {
          syncGlobalToPhysical();
        }
      }
    } else if (cursorCol == 2) {
      if (cursorRow == 0) {
        int m = (int)z.mode + direction;
        if (m < 0) m = 4;
        if (m > 4) m = 0;
        z.mode = (LightMode)m;
      }
      if (cursorRow == 1) z.lightCount = constrain(z.lightCount + direction, 0, 50);
      if (cursorRow == 2) z.scatter = constrain(z.scatter + direction, 0, 100);
    }

    if (currentZoneIdx == 3) {
      syncGlobalToPhysical();
    }
  }
}

void enterSleepMode() {
  Serial.println("System: Entering Light Sleep...");

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(10, 32, "Entering Sleep...");
  u8g2.sendBuffer();

  u8g2.setPowerSave(1);
  delay(50);

  waitForButtonRelease(BTN_BACK);
  waitForButtonRelease(BTN_CONFIRM);
  delay(BACK_DEBOUNCE_MS);

  esp_now_deinit();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(50);

#if defined(CONFIG_IDF_TARGET_ESP32C6) || defined(CONFIG_IDF_TARGET_ESP32C3)
  gpio_wakeup_enable((gpio_num_t)BTN_BACK, GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable((gpio_num_t)BTN_CONFIRM, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();
#else
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BTN_BACK, 0);
#endif

  while (true) {
    esp_light_sleep_start();

    bool backPressed = (digitalRead(BTN_BACK) == LOW);
    bool confirmPressed = (digitalRead(BTN_CONFIRM) == LOW);

    if (confirmPressed && !backPressed) {
      initWireless();

      if (currentZoneIdx < 3) {
        toggleSpecificZone(currentZoneIdx);
        broadcastSettings(currentZoneIdx);
      } else {
        toggleGlobalZones();
        broadcastSettings(3);
      }

      waitForButtonRelease(BTN_CONFIRM);
      esp_now_deinit();
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      delay(20);

#if defined(CONFIG_IDF_TARGET_ESP32C6) || defined(CONFIG_IDF_TARGET_ESP32C3)
      gpio_wakeup_enable((gpio_num_t)BTN_BACK, GPIO_INTR_LOW_LEVEL);
      gpio_wakeup_enable((gpio_num_t)BTN_CONFIRM, GPIO_INTR_LOW_LEVEL);
      esp_sleep_enable_gpio_wakeup();
#else
      esp_sleep_enable_ext0_wakeup((gpio_num_t)BTN_BACK, 0);
#endif
      continue;
    }

    initWireless();
    u8g2.setPowerSave(0);
    lastHeartbeatReceived = millis();
    lastUserActivity = millis();
    displayDirty = true;
    Serial.println("System: Woken up!");
    return;
  }
}

void handleButtons() {
  static bool lastPshState = HIGH;
  static bool lastBackState = HIGH;

  static unsigned long lastPshChange = 0;
  static unsigned long lastBackChange = 0;

  static bool confirmPressed = false;
  static unsigned long lastConfirmEdge = 0;
  static unsigned long confirmPressStart = 0;
  static bool confirmLongHandled = false;

  bool pshCurr = digitalRead(BTN_PSH);
  bool confirmCurr = digitalRead(BTN_CONFIRM);
  bool backCurr = digitalRead(BTN_BACK);
  unsigned long now = millis();

  if (pshCurr != lastPshState && (now - lastPshChange) > DEBOUNCE_MS) {
    lastPshChange = now;
    if (pshCurr == LOW) {
      noteUserActivity();
      if (inSettingsMenu) {
        if (settingsCursor == 0) {
          automaticLights = !automaticLights;
          broadcastSettings(3);
        } else if (settingsCursor == 1) {
          useFahrenheit = !useFahrenheit;
        } else if (settingsCursor == 2) {
          isEditing = !isEditing;
        }
      } else if (!isEditing && cursorRow == 3) {
        inSettingsMenu = true;
        settingsCursor = 0;
      }
      // Open custom color only when the cell currently shows SetClr (mode not RAINBOW/MUSIC)
      else if (!inCustomColorSubmenu &&
          !isEditing &&
          cursorRow == 1 &&
          cursorCol == 0 &&
          zones[currentZoneIdx].colorChoice == COLOR_CUSTOM &&
          zones[currentZoneIdx].mode != RAINBOW &&
          zones[currentZoneIdx].mode != MUSIC) {
        inCustomColorSubmenu = true;
        isEditing = false;
        cursorRow = 0;
        cursorCol = 0;
      } else {
        isEditing = !isEditing;
      }
      if (!inSettingsMenu) broadcastSettings(currentZoneIdx);
    }
    lastPshState = pshCurr;
  }

  if (!inCustomColorSubmenu && !inSettingsMenu) {
    if (!confirmPressed && confirmCurr == LOW && (now - lastConfirmEdge) > DEBOUNCE_MS) {
      noteUserActivity();
      confirmPressed = true;
      confirmPressStart = now;
      confirmLongHandled = false;
      lastConfirmEdge = now;
    }

    if (confirmPressed && confirmCurr == LOW && !confirmLongHandled && (now - confirmPressStart) >= CONFIRM_LONG_PRESS_MS) {
      noteUserActivity();
      confirmLongHandled = true;
      toggleGlobalZones();
      broadcastSettings(3);
    }

    if (confirmPressed && confirmCurr == HIGH && (now - lastConfirmEdge) > DEBOUNCE_MS) {
      noteUserActivity();
      if (!confirmLongHandled) {
        if (currentZoneIdx < 3) {
          toggleSpecificZone(currentZoneIdx);
          broadcastSettings(currentZoneIdx);
        } else {
          toggleGlobalZones();
          broadcastSettings(3);
        }
      }
      confirmPressed = false;
      lastConfirmEdge = now;
    }
  } else {
    confirmPressed = false;
    confirmLongHandled = false;
  }

  if (backCurr != lastBackState && (now - lastBackChange) > BACK_DEBOUNCE_MS) {
    lastBackChange = now;
    if (backCurr == LOW) {
      noteUserActivity();
      if (inSettingsMenu) {
        if (isEditing) {
          isEditing = false;
        } else {
          inSettingsMenu = false;
          cursorRow = 3;
          cursorCol = 0;
        }
      } else if (inCustomColorSubmenu) {
        if (isEditing) {
          isEditing = false;
        } else {
          inCustomColorSubmenu = false;
          cursorRow = 1;
          cursorCol = 0;
        }
        broadcastSettings(currentZoneIdx);
      } else if (isEditing) {
        isEditing = false;
        broadcastSettings(currentZoneIdx);
      } else {
        enterSleepMode();
      }
    }
    lastBackState = backCurr;
  }
}

void updateDisplay() {
  u8g2.clearBuffer();
  ZoneSettings &z = zones[currentZoneIdx];

  if (inSettingsMenu) {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(2, 10, "Settings");

    char buf[24];
    for (int row = 0; row < 3; row++) {
      int y = 21 + (row * 13);
      if (settingsCursor == row && isEditing) {
        u8g2.drawBox(0, y - 10, 128, 12);
        u8g2.setDrawColor(0);
      } else if (settingsCursor == row) {
        u8g2.drawFrame(0, y - 10, 128, 12);
        u8g2.setDrawColor(1);
      } else {
        u8g2.setDrawColor(1);
      }
      if (row == 0) snprintf(buf, sizeof(buf), "Auto lights: %s", automaticLights ? "ON" : "OFF");
      else if (row == 1) snprintf(buf, sizeof(buf), "Temperature: %c", useFahrenheit ? 'F' : 'C');
      else {
        float displayedOffset = useFahrenheit ? (temperatureOffsetC * 1.8f) : temperatureOffsetC;
        snprintf(buf, sizeof(buf), "Temp adjust: %+.1f%c", displayedOffset, useFahrenheit ? 'F' : 'C');
      }
      u8g2.drawStr(4, y, buf);
    }

    u8g2.setDrawColor(1);
    bool telemetryFresh = millis() - lastHeartbeatReceived < 5000;
    bool radarOk = telemetryFresh && (receivedTelemetryFlags & TELEMETRY_RADAR_CONNECTED);
    bool ahtOk = telemetryFresh && (receivedTelemetryFlags & TELEMETRY_AHT_CONNECTED);
    snprintf(buf, sizeof(buf), "Radar:%s AHT:%s", radarOk ? "OK" : "--", ahtOk ? "OK" : "--");
    u8g2.drawStr(2, 62, buf);
    u8g2.sendBuffer();
    return;
  }

  if (inCustomColorSubmenu) {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(2, 10, "Custom Color");

    char buf[16];
    for (int r = 0; r < 3; r++) {
      int y = 26 + (r * 14);
      bool selected = (cursorRow == r);
      if (selected) {
        if (isEditing) {
          u8g2.drawBox(0, y - 10, 80, 12);
          u8g2.setDrawColor(0);
        } else {
          u8g2.drawFrame(0, y - 10, 80, 12);
          u8g2.setDrawColor(1);
        }
      } else {
        u8g2.setDrawColor(1);
      }

      if (r == 0) snprintf(buf, sizeof(buf), "R:%u", z.customR);
      if (r == 1) snprintf(buf, sizeof(buf), "G:%u", z.customG);
      if (r == 2) snprintf(buf, sizeof(buf), "B:%u", z.customB);
      u8g2.drawStr(4, y, buf);
    }

    u8g2.setDrawColor(1);
    u8g2.drawStr(84, 56, "BACK");
    u8g2.sendBuffer();
    return;
  }

  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < COLS; c++) {
      int x = c * 43;
      int y = (r * 15) + 12;

      bool isSelected = (r == cursorRow && c == cursorCol);
      if (isSelected) {
        if (isEditing) {
          u8g2.drawBox(x, y - 10, 41, 14);
          u8g2.setDrawColor(0);
        } else {
          u8g2.drawFrame(x, y - 10, 41, 14);
          u8g2.setDrawColor(1);
        }
      } else {
        u8g2.setDrawColor(1);
      }

      // Zone icon cell (changes with selected zone)
      if (c == 1 && r == 1) {
        u8g2.drawXBMP(x + 16, y - 8, 8, 8, getSelectedZoneIcon(currentZoneIdx));
        continue;
      }

      // Brightness icon cell prefix
      if (c == 0 && r == 2) {
        u8g2.drawXBMP(x + 2, y - 8, 8, 8, ICON_SUN);
      }

      char buf[16];
      buf[0] = '\0';
      if (c == 0) {
        if (r == 0) snprintf(buf, sizeof(buf), colorNames[z.colorChoice]);
        if (r == 1) {
          if (z.mode == RAINBOW) snprintf(buf, sizeof(buf), "RSpd:%u", z.rainbowSpeed);
          else if (z.mode == MUSIC) snprintf(buf, sizeof(buf), "MSen:%u", z.musicSensitivity);
          else if (z.colorChoice == COLOR_CUSTOM) snprintf(buf, sizeof(buf), "[SetClr]");
        }
        if (r == 2) snprintf(buf, sizeof(buf), "%d", z.brightness);
      } else if (c == 2) {
        if (r == 0) snprintf(buf, sizeof(buf), "%s", modeNames[z.mode]);
        if (r == 1) snprintf(buf, sizeof(buf), "On:%u", z.lightCount);
        if (r == 2) snprintf(buf, sizeof(buf), "Sct:%d", z.scatter);
      }

      if (buf[0] != '\0') {
        int textX = (c == 0 && r == 2) ? (x + 12) : (x + 2);
        u8g2.drawStr(textX, y, buf);
      }
    }
  }

  bool connected = millis() - lastHeartbeatReceived < 5000;
  bool ahtOk = connected && (receivedTelemetryFlags & TELEMETRY_AHT_CONNECTED);
  bool presence = connected && (receivedTelemetryFlags & TELEMETRY_PRESENCE);
  if (cursorRow == 3) u8g2.drawFrame(0, 52, 128, 12);

  char telemetry[22];
  if (ahtOk) {
    float temperature = (receivedTemperatureCentiC / 100.0f) + temperatureOffsetC;
    if (useFahrenheit) temperature = (temperature * 9.0f / 5.0f) + 32.0f;
    snprintf(telemetry, sizeof(telemetry), "%.1f%c %u%% %c%s",
             temperature, useFahrenheit ? 'F' : 'C',
             (unsigned)(receivedHumidityCentiPct / 100),
             presence ? 'P' : '-', automaticLights ? " AUTO" : "");
  } else {
    snprintf(telemetry, sizeof(telemetry), "--.-%c --%% %c%s",
             useFahrenheit ? 'F' : 'C', presence ? 'P' : '-', automaticLights ? " AUTO" : "");
  }
  u8g2.drawStr(3, 62, telemetry);
  u8g2.drawXBMP(118, 54, 8, 8, connected ? ICON_WIFI_CONNECTED : ICON_DISCONNECTED_X);

  u8g2.sendBuffer();
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
}
#else
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
}
#endif

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void OnDataRecv(const esp_now_recv_info_t * recv_info, const uint8_t *incomingData, int len) {
#else
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
#endif
  if (len == sizeof(TelemetryPacket)) {
    TelemetryPacket packet;
    memcpy(&packet, incomingData, sizeof(packet));
    if (packet.magic != TELEMETRY_MAGIC) return;
    bool telemetryChanged =
      receivedTemperatureCentiC != packet.temperatureCentiC ||
      receivedHumidityCentiPct != packet.humidityCentiPct ||
      receivedTelemetryFlags != packet.flags;
    receivedTemperatureCentiC = packet.temperatureCentiC;
    receivedHumidityCentiPct = packet.humidityCentiPct;
    receivedTelemetryFlags = packet.flags;
    lastHeartbeatReceived = millis();
    if (telemetryChanged) displayDirty = true;
  } else if (len == 1 && incomingData[0] == 0xAB) {
    lastHeartbeatReceived = millis();
  }
}
