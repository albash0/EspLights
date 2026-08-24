#define FASTLED_INTERNAL
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <FastLED.h>
#include <esp_arduino_version.h>
#include <Wire.h>
#include <ld2410.h>

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #include <driver/i2s_std.h>
  #define USE_NEW_I2S 1
#else
  #include <driver/i2s.h>
  #define USE_NEW_I2S 0
  #define I2S_PORT I2S_NUM_0
#endif

#define NUM_ZONES       3
#define ZONE_MAX_LEDS   33
#define TOTAL_LEDS      (NUM_ZONES * ZONE_MAX_LEDS)
#define LED_PIN         5
#define LED_CHIPSET     WS2811
#define LED_COLOR_ORDER RGB

#define I2S_SCK         7
#define I2S_WS          10
#define I2S_SD          20
#define I2S_LR          21   // adjust to wiring: LOW=left, HIGH=right

// Optional environmental/presence sensors. Change these four pins to match wiring.
#define RADAR_RX_PIN    4    // ESP32 RX <- LD2410C TX
#define RADAR_TX_PIN    6    // ESP32 TX -> LD2410C RX
#define AHT_SDA_PIN     8
#define AHT_SCL_PIN     9
#define AHT10_ADDRESS   0x38

#define TELEMETRY_MAGIC 0xA17C
#define MUSIC_LINK_MAGIC 0xE35A
#define MUSIC_LINK_REQUEST 1
#define MUSIC_LINK_LEVEL 2
#define MUSIC_LINK_MIC_PRESENT 0x01
#define MUSIC_STREAM_INTERVAL_MS 33
#define MUSIC_REQUEST_TIMEOUT_MS 4000
#define TELEMETRY_RADAR_CONNECTED 0x01
#define TELEMETRY_AHT_CONNECTED   0x02
#define TELEMETRY_PRESENCE        0x04
#define RECEIVER_A 0
#define RECEIVER_L 1
#define RECEIVER_BOTH 2

enum LightMode { OFF, STABLE, RAINBOW, BREATHING, MUSIC };

struct ZoneSettings {
  uint8_t r = 180; uint8_t g = 50; uint8_t b = 250;
  uint8_t brightness = 150;
  LightMode mode = OFF;
  uint16_t lightCount = 30;
  uint8_t scatter = 0;
  uint8_t rainbowSpeed = 20;      // 0-255, controls how fast rainbow advances
  uint8_t musicSensitivity = 128; // 0-255, multiplies mic response in MUSIC mode
};

ZoneSettings zones[4];
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct __attribute__((packed)) ESPNowPacket {
  uint8_t targetReceiver;
  uint8_t targetZone; uint8_t r; uint8_t g; uint8_t b;
  uint8_t brightness; uint8_t mode; uint16_t lightCount; uint8_t scatter;
  uint8_t rainbowSpeed; uint8_t musicSensitivity;
  uint8_t automaticLights;
};

struct __attribute__((packed)) TelemetryPacket {
  uint16_t magic;
  int16_t temperatureCentiC;
  uint16_t humidityCentiPct;
  uint8_t flags;
};

struct __attribute__((packed)) MusicLinkPacket {
  uint16_t magic;
  uint32_t sessionId;
  uint32_t sequence;
  uint8_t messageType;
  uint8_t value;
  uint8_t flags;
};

CRGB leds[TOTAL_LEDS];
float volumeAmplitude = 0.0f;
float micEnvelope = 0.0f;
float micNoiseFloor = 0.0f;
bool micPresent = true;
uint8_t zoneFade[NUM_ZONES] = {0, 0, 0};
ld2410 radar;
bool radarConnected = false;
bool presenceDetected = false;
bool ahtConnected = false;
bool automaticLights = false;
float temperatureC = 0.0f;
float humidityPct = 0.0f;
const uint32_t RADAR_BAUD_RATES[] = {256000, 115200, 57600, 38400, 19200, 9600};
uint8_t radarBaudIndex = 0;
uint32_t lastRadarFrameMs = 0;
uint32_t lastRadarBaudChangeMs = 0;
uint32_t radarBytesSeen = 0;
uint32_t musicSessionId = 0;
uint32_t nextMusicSequence = 1;
volatile bool musicStreamRequested = false;
volatile uint32_t lastMusicRequestMs = 0;
uint32_t lightReceiverSessionId = 0;
uint32_t lastLightReceiverSequence = 0;

#if USE_NEW_I2S
i2s_chan_handle_t rx_chan = NULL;
#endif

void initI2S();
void readMic();
void renderLEDs();
void initOptionalSensors();
void updateOptionalSensors();
bool initAHT10();
bool readAHT10(float &temperature, float &humidity);
void sendTelemetry();
void serviceMusicStream();
uint32_t newSessionId();

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void OnDataRecv(const esp_now_recv_info_t * recv_info, const uint8_t *incomingData, int len) {
#else
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
#endif
  if (len == sizeof(MusicLinkPacket)) {
    MusicLinkPacket musicPacket;
    memcpy(&musicPacket, incomingData, sizeof(musicPacket));
    if (musicPacket.magic == MUSIC_LINK_MAGIC &&
        musicPacket.messageType == MUSIC_LINK_REQUEST &&
        musicPacket.sessionId != 0) {
      if (musicPacket.sessionId != lightReceiverSessionId) {
        lightReceiverSessionId = musicPacket.sessionId;
        lastLightReceiverSequence = 0;
      }
      if (musicPacket.sequence == 0 || musicPacket.sequence <= lastLightReceiverSequence) {
        Serial.println("Dropped duplicate/replayed music request from receiver L.");
        return;
      }
      lastLightReceiverSequence = musicPacket.sequence;
      musicStreamRequested = musicPacket.value != 0;
      lastMusicRequestMs = millis();
      return;
    }
  }

  if (len != sizeof(ESPNowPacket)) return;
  ESPNowPacket packet; memcpy(&packet, incomingData, sizeof(ESPNowPacket));
  if (packet.targetReceiver != RECEIVER_A && packet.targetReceiver != RECEIVER_BOTH) return;
  automaticLights = packet.automaticLights != 0;
  
  if (packet.targetZone <= 2) {
    zones[packet.targetZone].r = packet.r;
    zones[packet.targetZone].g = packet.g;
    zones[packet.targetZone].b = packet.b;
    zones[packet.targetZone].brightness = packet.brightness;
    zones[packet.targetZone].mode = (LightMode)packet.mode;
    zones[packet.targetZone].lightCount = packet.lightCount;
    zones[packet.targetZone].scatter = packet.scatter;
    zones[packet.targetZone].rainbowSpeed = packet.rainbowSpeed;
    zones[packet.targetZone].musicSensitivity = packet.musicSensitivity;
  } else if (packet.targetZone == 3) {
    for (int i = 0; i < 3; i++) {
      zones[i].r = packet.r;
      zones[i].g = packet.g;
      zones[i].b = packet.b;
      zones[i].brightness = packet.brightness;
      zones[i].mode = (LightMode)packet.mode;
      zones[i].lightCount = packet.lightCount;
      zones[i].scatter = packet.scatter;
      zones[i].rainbowSpeed = packet.rainbowSpeed;
      zones[i].musicSensitivity = packet.musicSensitivity;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("Booting receiver...");
  musicSessionId = newSessionId();

  pinMode(I2S_LR, OUTPUT);
  digitalWrite(I2S_LR, LOW);   // must match I2S_STD_SLOT_LEFT below
  Serial.printf("Mic channel select (L/R pin) = %s\n", digitalRead(I2S_LR) == LOW ? "LEFT" : "RIGHT");

  initI2S();
  initOptionalSensors();

  FastLED.addLeds<LED_CHIPSET, LED_PIN, LED_COLOR_ORDER>(leds, TOTAL_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setMaxPowerInVoltsAndMilliamps(12, 2000);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_now_register_recv_cb(OnDataRecv);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
  }
}

void loop() {
  updateOptionalSensors();
  readMic();
  serviceMusicStream();
  renderLEDs();
  
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 2000) {
    lastHeartbeat = millis();
    sendTelemetry();
  }
}

uint32_t newSessionId() {
  uint32_t id = 0;
  while (id == 0) id = esp_random();
  return id;
}

void initOptionalSensors() {
  Wire.begin(AHT_SDA_PIN, AHT_SCL_PIN);
  Wire.setClock(100000);
  ahtConnected = initAHT10();
  Serial.printf("AHT10: %s\n", ahtConnected ? "connected" : "not detected (continuing)");

  Serial1.begin(RADAR_BAUD_RATES[radarBaudIndex], SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  radar.begin(Serial1, false); // Do not wait or fail boot when the radar is absent.
  delay(20);
  radar.read();
  radarConnected = radar.isConnected();
  Serial.printf("LD2410C: %s\n", radarConnected ? "connected" : "not detected (continuing)");
}

bool initAHT10() {
  Wire.beginTransmission(AHT10_ADDRESS);
  if (Wire.endTransmission() != 0) return false;

  const uint8_t initCommand[] = {0xBE, 0x08, 0x00};
  Wire.beginTransmission(AHT10_ADDRESS);
  Wire.write(initCommand, sizeof(initCommand));
  if (Wire.endTransmission() != 0) return false;
  delay(10);
  return true;
}

bool readAHT10(float &temperature, float &humidity) {
  const uint8_t measureCommand[] = {0xAC, 0x33, 0x00};
  Wire.beginTransmission(AHT10_ADDRESS);
  Wire.write(measureCommand, sizeof(measureCommand));
  if (Wire.endTransmission() != 0) return false;
  delay(85);

  if (Wire.requestFrom((uint8_t)AHT10_ADDRESS, (uint8_t)6) != 6) return false;
  uint8_t data[6];
  for (uint8_t i = 0; i < sizeof(data); i++) data[i] = Wire.read();
  if (data[0] & 0x80) return false;

  uint32_t rawHumidity = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
  uint32_t rawTemperature = ((uint32_t)(data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];
  humidity = constrain((rawHumidity * 100.0f) / 1048576.0f, 0.0f, 100.0f);
  temperature = ((rawTemperature * 200.0f) / 1048576.0f) - 50.0f;
  return true;
}

void updateOptionalSensors() {
  radarBytesSeen += Serial1.available();
  radar.read();
  static unsigned long lastAhtRead = 0;
  static unsigned long lastAhtProbe = 0;
  static unsigned long lastSensorPrint = 0;
  unsigned long now = millis();

  if (radar.isConnected()) {
    lastRadarFrameMs = now;
    radarConnected = true;
    presenceDetected = radar.presenceDetected();
  } else if (now - lastRadarFrameMs > 3000) {
    radarConnected = false;
    presenceDetected = false;
    if (now - lastRadarBaudChangeMs > 1500) {
      lastRadarBaudChangeMs = now;
      radarBaudIndex = (radarBaudIndex + 1) % (sizeof(RADAR_BAUD_RATES) / sizeof(RADAR_BAUD_RATES[0]));
      Serial1.updateBaudRate(RADAR_BAUD_RATES[radarBaudIndex]);
      Serial.printf("LD2410C: trying %lu baud on RX=%d TX=%d\n",
                    (unsigned long)RADAR_BAUD_RATES[radarBaudIndex], RADAR_RX_PIN, RADAR_TX_PIN);
    }
  }

  if (!ahtConnected && now - lastAhtProbe >= 10000) {
    lastAhtProbe = now;
    ahtConnected = initAHT10();
  }

  if (ahtConnected && now - lastAhtRead >= 2000) {
    lastAhtRead = now;
    if (!readAHT10(temperatureC, humidityPct)) ahtConnected = false;
  }

  if (now - lastSensorPrint >= 2000) {
    lastSensorPrint = now;
    Serial.printf("SENSORS radar=%s presence=%s baud=%lu uartBytes=%lu AHT=%s temp=%.2fC humidity=%.2f%%\n",
                  radarConnected ? "OK" : "--", presenceDetected ? "YES" : "NO",
                  (unsigned long)RADAR_BAUD_RATES[radarBaudIndex],
                  (unsigned long)radarBytesSeen,
                  ahtConnected ? "OK" : "--", temperatureC, humidityPct);
  }
}

void sendTelemetry() {
  TelemetryPacket packet = {};
  packet.magic = TELEMETRY_MAGIC;
  packet.temperatureCentiC = ahtConnected ? (int16_t)roundf(temperatureC * 100.0f) : 0;
  packet.humidityCentiPct = ahtConnected ? (uint16_t)roundf(humidityPct * 100.0f) : 0;
  if (radarConnected) packet.flags |= TELEMETRY_RADAR_CONNECTED;
  if (ahtConnected) packet.flags |= TELEMETRY_AHT_CONNECTED;
  if (presenceDetected) packet.flags |= TELEMETRY_PRESENCE;
  esp_now_send(broadcastAddress, (uint8_t *)&packet, sizeof(packet));
}

void serviceMusicStream() {
  uint32_t now = millis();
  if (musicStreamRequested && now - lastMusicRequestMs > MUSIC_REQUEST_TIMEOUT_MS) {
    musicStreamRequested = false;
    Serial.println("Receiver L music request expired; stopping stream.");
  }
  if (!musicStreamRequested) return;

  static uint32_t lastMusicSendMs = 0;
  if (now - lastMusicSendMs < MUSIC_STREAM_INTERVAL_MS) return;
  lastMusicSendMs = now;

  MusicLinkPacket packet = {};
  packet.magic = MUSIC_LINK_MAGIC;
  packet.sessionId = musicSessionId;
  packet.sequence = nextMusicSequence++;
  packet.messageType = MUSIC_LINK_LEVEL;
  packet.value = micPresent ? (uint8_t)roundf(constrain(volumeAmplitude, 0.0f, 1.0f) * 255.0f) : 0;
  if (micPresent) packet.flags |= MUSIC_LINK_MIC_PRESENT;

  if (nextMusicSequence == 0) {
    musicSessionId = newSessionId();
    nextMusicSequence = 1;
  }
  esp_now_send(broadcastAddress, (uint8_t *)&packet, sizeof(packet));
}

void initI2S() {
#if USE_NEW_I2S
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  if (i2s_new_channel(&chan_cfg, NULL, &rx_chan) == ESP_OK) {
    i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
      .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
      .gpio_cfg = {
        .bclk = (gpio_num_t)I2S_SCK,
        .ws   = (gpio_num_t)I2S_WS,
        .din  = (gpio_num_t)I2S_SD
      }
    };
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
    esp_err_t err = i2s_channel_init_std_mode(rx_chan, &std_cfg);
    if (err == ESP_OK) {
      i2s_channel_enable(rx_chan);
      Serial.println("I2S mic init OK (new driver).");
    } else {
      Serial.printf("I2S init failed: %d\n", (int)err);
    }
  } else {
    Serial.println("I2S channel creation failed.");
  }
#else
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .dma_buf_count = 2,
    .dma_buf_len = 32,
    .use_apll = false
  };
  esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err == ESP_OK) {
    i2s_set_pin(I2S_PORT, &(i2s_pin_config_t){
      .bck_io_num = I2S_SCK,
      .ws_io_num = I2S_WS,
      .data_in_num = I2S_SD
    });
    Serial.println("I2S mic init OK (legacy driver).");
  } else {
    Serial.printf("I2S init failed: %d\n", (int)err);
  }
#endif
}

void readMic() {
  int32_t buffer[16];
  size_t read = 0;

#if USE_NEW_I2S
  i2s_channel_read(rx_chan, buffer, sizeof(buffer), &read, 0);
#else
  i2s_read(I2S_PORT, buffer, sizeof(buffer), &read, 0);
#endif

  static unsigned long lastMicPrint = 0;

  if (read > 0) {
    float sum = 0.0f;
    int samples = read / sizeof(int32_t);

    for (int i = 0; i < samples; i++) {
      sum += abs(buffer[i] >> 16);
    }

    float level = (samples > 0) ? (sum / samples) : 0.0f;

    // Track a very slow noise floor and keep it conservative so normal music is not canceled out.
    if (micNoiseFloor == 0.0f) {
      micNoiseFloor = level;
    } else {
      micNoiseFloor = (micNoiseFloor * 0.998f) + (level * 0.002f);
    }

    float signal = level - (micNoiseFloor * 1.05f);
    if (signal < 0.0f) signal = 0.0f;

    // Faster attack + slower release gives better visible response to beats.
    float attack = 0.28f;
    float release = 0.08f;
    float blend = (signal > micEnvelope) ? attack : release;
    micEnvelope = (micEnvelope * (1.0f - blend)) + (signal * blend);

    // Map to 0..1 with more gain so response works at normal listening distance.
    float boosted = (micEnvelope - 10.0f) / 700.0f;
    volumeAmplitude = constrain(boosted, 0.0f, 1.0f);
    micPresent = true;

    if (millis() - lastMicPrint > 500) {
      lastMicPrint = millis();
      Serial.printf(
        "MIC OK: read=%u bytes samples=%d raw=%.1f floor=%.1f env=%.1f amp=%.3f lr=%s mode=%d\n",
        (unsigned)read,
        samples,
        level,
        micNoiseFloor,
        micEnvelope,
        volumeAmplitude,
        (digitalRead(I2S_LR) == LOW) ? "LEFT" : "RIGHT",
        (int)zones[0].mode
      );
    }
  } else {
    micPresent = false;
    if (millis() - lastMicPrint > 1000) {
      lastMicPrint = millis();
      Serial.println("MIC: no data read");
    }
  }
}

void renderLEDs() {
  static uint8_t hue = 0;
  static unsigned long lastHueMs = 0;
  unsigned long now = millis();
  // A missing/disconnected radar never suppresses the lights.
  bool automaticOff = automaticLights && radarConnected && !presenceDetected;

  // Compute an average rainbow speed across zones that are in RAINBOW mode.
  uint32_t sumSpeed = 0;
  int speedCount = 0;
  for (int zz = 0; zz < NUM_ZONES; zz++) {
    if (zones[zz].mode == RAINBOW) { sumSpeed += zones[zz].rainbowSpeed; speedCount++; }
  }
  uint8_t avgSpeed = speedCount ? (uint8_t)(sumSpeed / speedCount) : 20;

  // Advance hue according to avgSpeed and elapsed time (base tick ~20ms)
  unsigned long dt = now - lastHueMs;
  if (dt >= 20) {
    lastHueMs = now;
    uint8_t step = max(1, avgSpeed / 16); // map 0-255 -> ~0-15
    hue += step;
  }

  fadeToBlackBy(leds, TOTAL_LEDS, 18);

  for (int z = 0; z < NUM_ZONES; z++) {
    ZoneSettings &s = zones[z];

    if (s.mode == OFF || automaticOff) {
      zoneFade[z] = qsub8(zoneFade[z], 18);
      continue;
    }

    zoneFade[z] = qadd8(zoneFade[z], 10);

    uint8_t br = s.brightness;
    if (s.mode == BREATHING) br = beatsin8(40, 50, s.brightness);

    int skip = s.scatter;
    int count = 0;

    for (int i = 0; i < s.lightCount; i++) {
      int idx = z * ZONE_MAX_LEDS + count;
      if (idx >= (z + 1) * ZONE_MAX_LEDS) break;

      CRGB target;
      if (s.mode == RAINBOW) {
        // allow per-zone offset but use global hue speed
        target = CHSV(hue + (i * 5), 200, br);
      } else if (s.mode == MUSIC) {
        // apply music sensitivity multiplier (centered ~128 -> 1.0)
        float sens = (float)s.musicSensitivity / 128.0f;
        float shaped = sqrtf(volumeAmplitude); // less aggressive compression than squaring
        float v = shaped * (float)br * sens;
        uint8_t musicV = (uint8_t)constrain(v, 0.0f, 255.0f);
        if (musicV < 3) musicV = 0;
        target = CHSV(hue, 200, musicV);
      } else {
        target = CRGB(s.r, s.g, s.b);
        target.nscale8_video(br);
      }

      target.nscale8_video(zoneFade[z]);
      nblend(leds[idx], target, 48);

      count += (1 + skip);
    }
  }

  FastLED.show();
}
