#define FASTLED_INTERNAL
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_system.h>
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
#define MUSIC_LINK_MAGIC 0xE35A
#define MUSIC_LINK_REQUEST 1
#define MUSIC_LINK_LEVEL 2
#define MUSIC_LINK_MIC_PRESENT 0x01
#define MUSIC_REQUEST_INTERVAL_MS 1500
#define MUSIC_LEVEL_TIMEOUT_MS 500

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

struct __attribute__((packed)) MusicLinkPacket {
  uint16_t magic;
  uint32_t sessionId;
  uint32_t sequence;
  uint8_t messageType;
  uint8_t value;
  uint8_t flags;
};

ZoneSettings zones[NUM_ZONES];
CRGB leds[TOTAL_LEDS];
uint8_t zoneFade[NUM_ZONES] = {0, 0, 0};
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint32_t musicSessionId = 0;
uint32_t nextMusicRequestSequence = 1;
uint32_t primaryReceiverSessionId = 0;
uint32_t lastPrimaryReceiverSequence = 0;
volatile bool musicRequested = false;
volatile bool musicRequestDirty = false;
volatile uint8_t receivedMusicLevel = 0;
volatile bool receivedMicPresent = false;
volatile uint32_t lastMusicLevelMs = 0;

uint32_t newSessionId() {
  uint32_t id = 0;
  while (id == 0) id = esp_random();
  return id;
}

bool anyZoneUsesMusic() {
  for (int i = 0; i < NUM_ZONES; i++) {
    if (zones[i].mode == MUSIC) return true;
  }
  return false;
}

void updateMusicRequestState() {
  bool requested = anyZoneUsesMusic();
  if (requested != musicRequested) {
    musicRequested = requested;
    musicRequestDirty = true;
  }
}

void serviceMusicRequest() {
  uint32_t now = millis();
  bool sendNow = musicRequestDirty;
  if (sendNow) musicRequestDirty = false;

  static uint32_t lastRequestMs = 0;
  if (!sendNow && (!musicRequested || now - lastRequestMs < MUSIC_REQUEST_INTERVAL_MS)) return;
  lastRequestMs = now;

  MusicLinkPacket packet = {};
  packet.magic = MUSIC_LINK_MAGIC;
  packet.sessionId = musicSessionId;
  packet.sequence = nextMusicRequestSequence++;
  packet.messageType = MUSIC_LINK_REQUEST;
  packet.value = musicRequested ? 1 : 0;
  if (nextMusicRequestSequence == 0) {
    musicSessionId = newSessionId();
    nextMusicRequestSequence = 1;
  }
  esp_now_send(broadcastAddress, (uint8_t *)&packet, sizeof(packet));
}

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
  updateMusicRequestState();
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void OnDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *incomingData, int len) {
#else
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
#endif
  if (len == sizeof(MusicLinkPacket)) {
    MusicLinkPacket musicPacket;
    memcpy(&musicPacket, incomingData, sizeof(musicPacket));
    if (musicPacket.magic == MUSIC_LINK_MAGIC &&
        musicPacket.messageType == MUSIC_LINK_LEVEL &&
        musicPacket.sessionId != 0) {
      if (musicPacket.sessionId != primaryReceiverSessionId) {
        primaryReceiverSessionId = musicPacket.sessionId;
        lastPrimaryReceiverSequence = 0;
      }
      if (musicPacket.sequence == 0 || musicPacket.sequence <= lastPrimaryReceiverSequence) {
        Serial.println("Dropped duplicate/replayed music level from receiver A.");
        return;
      }
      lastPrimaryReceiverSequence = musicPacket.sequence;
      receivedMusicLevel = musicPacket.value;
      receivedMicPresent = (musicPacket.flags & MUSIC_LINK_MIC_PRESENT) != 0;
      lastMusicLevelMs = millis();
      return;
    }
  }

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
  musicSessionId = newSessionId();

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
  serviceMusicRequest();

  uint8_t remoteMusicLevel = receivedMusicLevel;
  bool remoteMusicAvailable = receivedMicPresent && now - lastMusicLevelMs <= MUSIC_LEVEL_TIMEOUT_MS;

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
      if (remoteMusicAvailable) {
        float normalized = (float)remoteMusicLevel / 255.0f;
        float sensitivity = (float)s.musicSensitivity / 128.0f;
        float level = sqrtf(normalized) * (float)s.brightness * sensitivity;
        brightness = (uint8_t)constrain(level, 0.0f, 255.0f);
        if (brightness < 3) brightness = 0;
      } else {
        uint8_t bpm = map(s.musicSensitivity, 0, 255, 20, 120);
        brightness = beatsin8(bpm, 0, s.brightness);
      }
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
