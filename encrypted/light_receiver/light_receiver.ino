#define FASTLED_INTERNAL
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <FastLED.h>
#include <esp_arduino_version.h>
#include "secrets.h"

// One physical strip split into the same three logical zones as receiver A.
#define NUM_ZONES       3
#define ZONE_MAX_LEDS   33
#define TOTAL_LEDS      (NUM_ZONES * ZONE_MAX_LEDS)
#define LED_PIN         5
#define LED_CHIPSET     WS2811
#define LED_COLOR_ORDER RGB

#define COMMAND_MAGIC   0xC041
#define MUSIC_LINK_MAGIC 0xE35A
#define MUSIC_LINK_REQUEST 1
#define MUSIC_LINK_LEVEL 2
#define MUSIC_LINK_MIC_PRESENT 0x01
#define MUSIC_REQUEST_INTERVAL_MS 1500
#define MUSIC_LEVEL_TIMEOUT_MS 500
#define RECEIVER_A      0
#define RECEIVER_L      1
#define RECEIVER_BOTH   2

#ifndef RECEIVER_LINK_CONFIGURED
  #define RECEIVER_LINK_CONFIGURED 0
#endif

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
  uint16_t magic;
  uint32_t sessionId;
  uint32_t sequence;
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

struct ControllerReplayState {
  uint32_t sessionId;
  uint32_t lastSequence;
};

struct MusicReplayState {
  uint32_t sessionId;
  uint32_t lastSequence;
};

static_assert(CONTROLLER_PEER_COUNT > 0, "Configure at least one controller peer.");
static_assert(
  CONTROLLER_PEER_COUNT + RECEIVER_LINK_CONFIGURED <= ESP_NOW_MAX_ENCRYPT_PEER_NUM,
  "Too many encrypted controller/receiver peers for this ESP-NOW build."
);

ZoneSettings zones[NUM_ZONES];
CRGB leds[TOTAL_LEDS];
uint8_t zoneFade[NUM_ZONES] = {0, 0, 0};
ControllerReplayState controllerReplayStates[CONTROLLER_PEER_COUNT] = {};
MusicReplayState primaryReceiverReplayState = {};
uint32_t musicSessionId = 0;
uint32_t nextMusicRequestSequence = 1;
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
#if RECEIVER_LINK_CONFIGURED
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
  esp_now_send(PRIMARY_RECEIVER_ADDRESS, (uint8_t *)&packet, sizeof(packet));
#endif
}

int findControllerPeer(const uint8_t *address) {
  if (address == nullptr) return -1;
  for (size_t i = 0; i < CONTROLLER_PEER_COUNT; i++) {
    if (memcmp(address, CONTROLLER_PEERS[i].address, 6) == 0) return (int)i;
  }
  return -1;
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
  const uint8_t *sourceAddress = recvInfo == nullptr ? nullptr : recvInfo->src_addr;
#else
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  const uint8_t *sourceAddress = mac;
#endif
#if RECEIVER_LINK_CONFIGURED
  if (sourceAddress != nullptr && memcmp(sourceAddress, PRIMARY_RECEIVER_ADDRESS, 6) == 0) {
    if (len != sizeof(MusicLinkPacket)) return;
    MusicLinkPacket packet;
    memcpy(&packet, incomingData, sizeof(packet));
    if (packet.magic != MUSIC_LINK_MAGIC ||
        packet.messageType != MUSIC_LINK_LEVEL ||
        packet.sessionId == 0) return;

    if (packet.sessionId != primaryReceiverReplayState.sessionId) {
      primaryReceiverReplayState.sessionId = packet.sessionId;
      primaryReceiverReplayState.lastSequence = 0;
    }
    if (packet.sequence == 0 || packet.sequence <= primaryReceiverReplayState.lastSequence) {
      Serial.println("Dropped duplicate/replayed music level from receiver A.");
      return;
    }
    primaryReceiverReplayState.lastSequence = packet.sequence;
    receivedMusicLevel = packet.value;
    receivedMicPresent = (packet.flags & MUSIC_LINK_MIC_PRESENT) != 0;
    lastMusicLevelMs = millis();
    return;
  }
#endif

  int controllerIndex = findControllerPeer(sourceAddress);
  if (controllerIndex < 0 || len != sizeof(ESPNowPacket)) return;

  ESPNowPacket packet;
  memcpy(&packet, incomingData, sizeof(packet));
  if (packet.magic != COMMAND_MAGIC || packet.sessionId == 0) return;
  if (packet.targetReceiver != RECEIVER_L && packet.targetReceiver != RECEIVER_BOTH) return;

  ControllerReplayState &replay = controllerReplayStates[controllerIndex];
  if (packet.sessionId != replay.sessionId) {
    replay.sessionId = packet.sessionId;
    replay.lastSequence = 0;
  }
  if (packet.sequence == 0 || packet.sequence <= replay.lastSequence) {
    Serial.printf("Dropped duplicate/replayed packet from controller %d.\n", controllerIndex + 1);
    return;
  }
  replay.lastSequence = packet.sequence;
  applyPacket(packet);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("Booting encrypted light-only receiver L...");
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
  if (esp_now_set_pmk(ESPNOW_PMK) != ESP_OK) {
    Serial.println("Failed to set ESP-NOW primary key.");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  for (size_t i = 0; i < CONTROLLER_PEER_COUNT; i++) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, CONTROLLER_PEERS[i].address, 6);
    peerInfo.channel = 1;
    peerInfo.ifidx = WIFI_IF_STA;
    peerInfo.encrypt = true;
    memcpy(peerInfo.lmk, CONTROLLER_PEERS[i].lmk, sizeof(peerInfo.lmk));
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.printf("Failed to register controller %u.\n", (unsigned)(i + 1));
    }
  }
#if RECEIVER_LINK_CONFIGURED
  esp_now_peer_info_t primaryReceiverPeer = {};
  memcpy(primaryReceiverPeer.peer_addr, PRIMARY_RECEIVER_ADDRESS, 6);
  primaryReceiverPeer.channel = 1;
  primaryReceiverPeer.ifidx = WIFI_IF_STA;
  primaryReceiverPeer.encrypt = true;
  memcpy(primaryReceiverPeer.lmk, RECEIVER_LINK_LMK, sizeof(primaryReceiverPeer.lmk));
  if (esp_now_add_peer(&primaryReceiverPeer) != ESP_OK) {
    Serial.println("Failed to register encrypted receiver A music peer.");
  } else {
    Serial.println("Receiver A-to-L encrypted music link enabled.");
  }
#else
  Serial.println("Receiver A-to-L music link not configured; Music will use fallback pulse.");
#endif
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
