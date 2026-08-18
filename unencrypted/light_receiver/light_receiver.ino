#define FASTLED_INTERNAL
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <FastLED.h>
#include <esp_arduino_version.h>

// One physical strip split into the same three logical zones as receiver A.
#define NUM_ZONES       3
#define ZONE_MAX_LEDS   33
#define TOTAL_LEDS      (NUM_ZONES * ZONE_MAX_LEDS)
#define LED_PIN         5
#define LED_CHIPSET     WS2811
#define LED_COLOR_ORDER RGB

#define RECEIVER_A      0
#define RECEIVER_L      1
#define RECEIVER_BOTH   2

enum LightMode { OFF, STABLE, RAINBOW, BREATHING, MUSIC };

struct ZoneSettings {
  uint8_t r = 180;
  uint8_t g = 50;
  uint8_t b = 250;
  uint8_t brightness = 150;
  LightMode mode = OFF;
  uint16_t lightCount = 30;
  uint8_t scatter = 0;
  uint8_t rainbowSpeed = 20;
  uint8_t musicSensitivity = 128;
};

struct __attribute__((packed)) ESPNowPacket {
  uint8_t targetReceiver;
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

ZoneSettings zones[NUM_ZONES];
CRGB leds[TOTAL_LEDS];
uint8_t zoneFade[NUM_ZONES] = {0, 0, 0};
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void applyPacket(const ESPNowPacket &packet) {
  if (packet.mode > MUSIC) return;

  auto applyToZone = [&packet](ZoneSettings &zone) {
    zone.r = packet.r;
    zone.g = packet.g;
    zone.b = packet.b;
    zone.brightness = packet.brightness;
    zone.mode = (LightMode)packet.mode;
    zone.lightCount = packet.lightCount;
    zone.scatter = packet.scatter;
    zone.rainbowSpeed = packet.rainbowSpeed;
    zone.musicSensitivity = packet.musicSensitivity;
  };

  if (packet.targetZone < NUM_ZONES) {
    applyToZone(zones[packet.targetZone]);
  } else if (packet.targetZone == NUM_ZONES) {
    for (int i = 0; i < NUM_ZONES; i++) applyToZone(zones[i]);
  }
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void OnDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *incomingData, int len) {
#else
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
#endif
  if (len != sizeof(ESPNowPacket)) return;
  ESPNowPacket packet;
  memcpy(&packet, incomingData, sizeof(packet));
  if (packet.targetReceiver != RECEIVER_L && packet.targetReceiver != RECEIVER_BOTH) return;
  applyPacket(packet);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("Booting light-only receiver L...");

  FastLED.addLeds<LED_CHIPSET, LED_PIN, LED_COLOR_ORDER>(leds, TOTAL_LEDS)
    .setCorrection(TypicalLEDStrip);
  FastLED.setMaxPowerInVoltsAndMilliamps(12, 2000);
  FastLED.clear(true);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  Serial.printf("Receiver L station MAC: %s\n", WiFi.macAddress().c_str());
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed.");
    return;
  }
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void loop() {
  static uint8_t hue = 0;
  static unsigned long lastHueMs = 0;
  unsigned long now = millis();

  uint32_t sumSpeed = 0;
  int speedCount = 0;
  for (int z = 0; z < NUM_ZONES; z++) {
    if (zones[z].mode == RAINBOW) {
      sumSpeed += zones[z].rainbowSpeed;
      speedCount++;
    }
  }
  uint8_t avgSpeed = speedCount ? (uint8_t)(sumSpeed / speedCount) : 20;
  if (now - lastHueMs >= 20) {
    lastHueMs = now;
    hue += max(1, avgSpeed / 16);
  }

  fadeToBlackBy(leds, TOTAL_LEDS, 18);
  for (int z = 0; z < NUM_ZONES; z++) {
    ZoneSettings &s = zones[z];
    if (s.mode == OFF) {
      zoneFade[z] = qsub8(zoneFade[z], 18);
      continue;
    }
    zoneFade[z] = qadd8(zoneFade[z], 10);

    uint8_t brightness = s.brightness;
    if (s.mode == BREATHING) brightness = beatsin8(40, 50, s.brightness);
    if (s.mode == MUSIC) {
      uint8_t bpm = map(s.musicSensitivity, 0, 255, 20, 120);
      brightness = beatsin8(bpm, 0, s.brightness);
    }

    int physicalOffset = 0;
    for (int i = 0; i < s.lightCount; i++) {
      int ledIndex = z * ZONE_MAX_LEDS + physicalOffset;
      if (ledIndex >= (z + 1) * ZONE_MAX_LEDS) break;

      CRGB target;
      if (s.mode == RAINBOW || s.mode == MUSIC) {
        target = CHSV(hue + (i * 5), 200, brightness);
      } else {
        target = CRGB(s.r, s.g, s.b);
        target.nscale8_video(brightness);
      }
      target.nscale8_video(zoneFade[z]);
      nblend(leds[ledIndex], target, 48);
      physicalOffset += 1 + s.scatter;
    }
  }
  FastLED.show();
  delay(5);
}
