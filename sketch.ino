// BLE sensor node (SIMULATED RADIO) - Wokwi
// Wokwi does not emulate Bluetooth, so the BLE advertisement is built exactly
// like a real one but is carried over WiFi/MQTT. RSSI and packet loss come
// from a log-distance path-loss model. The potentiometer sets the distance.

#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

// ---------- Config ----------
const char*    WIFI_SSID = "Wokwi-GUEST";
const char*    WIFI_PASS = "";
const char*    MQTT_HOST = "broker.hivemq.com";
const uint16_t MQTT_PORT = 1883;
// Public broker: make this unique (e.g. add your name) so others don't clash.
const char*    TOPIC_ADV = "CHANGE-ME-lecture/ble/adv";

const uint32_t ADV_INTERVAL_MS = 1000;   // try 100, 500, 1000, 2000
const float    TX_POWER_1M     = -59.0;  // RSSI at 1 m
const float    PATH_LOSS_EXP   = 2.2;    // 2.0 = free space, 3-4 = indoors/obstacles
const float    NOISE_SIGMA     = 3.0;    // dB of random fading
const float    LOSS_START_DBM  = -85.0;  // packets begin to drop below this
const float    LOSS_FULL_DBM   = -97.0;  // all packets lost below this

#define DHT_PIN 15
#define POT_PIN 34
#define DHT_TYPE DHT22

DHT dht(DHT_PIN, DHT_TYPE);
WiFiClient wifi;
PubSubClient mqtt(wifi);

uint16_t seq = 0;
uint8_t  battery = 100;
uint32_t lastAdv = 0;

float gaussian(float sigma) {
  float u1 = random(1, 10000) / 10000.0f;
  float u2 = random(1, 10000) / 10000.0f;
  return sigma * sqrtf(-2.0f * logf(u1)) * cosf(2.0f * PI * u2);
}

float distanceFromPot() {               // 1 m .. 60 m
  return 1.0f + (analogRead(POT_PIN) / 4095.0f) * 59.0f;
}

float rssiAt(float d) {
  return TX_POWER_1M - 10.0f * PATH_LOSS_EXP * log10f(d) + gaussian(NOISE_SIGMA);
}

bool packetSurvives(float rssi) {
  if (rssi >= LOSS_START_DBM) return true;
  if (rssi <= LOSS_FULL_DBM)  return false;
  float pLoss = (LOSS_START_DBM - rssi) / (LOSS_START_DBM - LOSS_FULL_DBM);
  return random(0, 1000) / 1000.0f >= pLoss;
}

void connectWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.print("WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS, 6);
  while (WiFi.status() != WL_CONNECTED) { delay(250); Serial.print("."); }
  Serial.println(" connected");
}

void connectMqtt() {
  while (!mqtt.connected()) {
    String id = "node-" + String(random(0xffff), HEX);
    Serial.print("MQTT...");
    if (mqtt.connect(id.c_str())) Serial.println(" connected");
    else { Serial.println(" retry"); delay(1000); }
  }
}

void setup() {
  Serial.begin(115200);
  dht.begin();
  randomSeed(esp_random());
  connectWifi();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  connectMqtt();
  Serial.println("Node ready. Turn the potentiometer to change distance.");
}

void loop() {
  connectWifi();
  connectMqtt();
  mqtt.loop();

  if (millis() - lastAdv < ADV_INTERVAL_MS) return;
  lastAdv = millis();

  float temp = dht.readTemperature();
  float hum  = dht.readHumidity();
  if (isnan(temp)) temp = 25.0;
  if (isnan(hum))  hum  = 50.0;

  // BLE manufacturer-specific advertising data (9 bytes)
  // [0-1] company ID 0xFFFF (test)  [2-3] temp x100  [4-5] hum x100
  // [6] battery %                   [7-8] sequence number
  int16_t  t = (int16_t)(temp * 100);
  uint16_t h = (uint16_t)(hum * 100);
  uint8_t pkt[9] = {0xFF, 0xFF,
                    (uint8_t)(t & 0xFF), (uint8_t)(t >> 8),
                    (uint8_t)(h & 0xFF), (uint8_t)(h >> 8),
                    battery,
                    (uint8_t)(seq & 0xFF), (uint8_t)(seq >> 8)};

  float dist = distanceFromPot();
  float rssi = rssiAt(dist);
  bool  sent = packetSurvives(rssi);

  Serial.printf("seq=%u dist=%.1fm rssi=%.0fdBm %s\n",
                seq, dist, rssi, sent ? "SENT" : "LOST (radio)");

  if (sent) {
    char hex[2 * 9 + 1];
    for (int i = 0; i < 9; i++) sprintf(&hex[i * 2], "%02X", pkt[i]);
    char msg[48];
    snprintf(msg, sizeof(msg), "%s,%d", hex, (int)rssi);
    mqtt.publish(TOPIC_ADV, msg);
  }

  seq++;
  if (seq % 20 == 0 && battery > 0) battery--;   // slow fake battery drain
}