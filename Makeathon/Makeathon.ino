/*
  Smart Office + Serial Relay Control (Reversed Fan & PIR Logic)
  ESP32 + MQ135 + DHT11 + PIR + LDR + OLED + 5 Relays + LED
  Firebase Realtime DB integration:
   - Writes time-series under /sensor_readings/sensor_<ms>
   - Updates latest snapshot under /state
   - Polls /controls every 2s and applies commands
  NOTE: replace WIFI and FIREBASE values below with your project values.
*/

#include <WiFi.h>
#include <FirebaseESP32.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "DHT.h"

// ---------- WIFI & FIREBASE CONFIG (replace these) ----------
const char* ssid = "mdkaif";
const char* password = "123456789";

// e.g. "your-project-default-rtdb.firebaseio.com"
#define FIREBASE_HOST "iotmakethon123-default-rtdb.firebaseio.com"
// legacy DB secret / token (for prototyping only)
#define FIREBASE_AUTH "EkPsSjXqlDLwqG8BJRg89YyO5eZUNSHNG6168tED"

FirebaseData fbdo;
FirebaseConfig config;
FirebaseAuth auth;
FirebaseData streamDo; // not used for streams here, but keep a second handle

// ------------------ OLED CONFIG ------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ------------------ MQ135 AIR QUALITY ------------------
#define MQ_PIN 34
#define PURIFIER_RELAY_PIN 26
const int SAMPLE_INTERVAL_MS = 1500;
const int NUM_SAMPLES = 8;
int samples[NUM_SAMPLES];
int sampleIndex = 0;
int AQ_THRESHOLD = 700;

// ------------------ DHT11 HEAT CONTROLLER ------------------
#define DHTPIN 27
#define DHTTYPE DHT11
#define FAN_RELAY_PIN 25
DHT dht(DHTPIN, DHTTYPE);
float TEMP_THRESHOLD = 30.0;
float HYSTERESIS = 2.0;
bool fanState = false; // true = fan ON, false = OFF

// ------------------ PIR LIGHT CONTROL ------------------
#define PIR_PIN 14
#define LIGHT_RELAY_PIN 13
bool pirState = false;

// ------------------ LDR LED LIGHT CONTROL (REVERSED) ------------------
#define LDR_PIN 35
#define LED_PIN 15
const int LDR_THRESHOLD = 1500;
bool ledState = false;

// ------------------ MANUAL SERIAL RELAY MODULE ------------------
int RELAY_LEFT = 2;
int RELAY_RIGHT = 4;
String dataSerial = "";

// ------------------ OLED SCREEN SHUFFLE ------------------
unsigned long lastScreenMillis = 0;
const int SCREEN_INTERVAL_MS = 4000;
int currentScreen = 0;
const int NUM_SCREENS = 3;

// ---------- Firebase & sensor timing ----------
unsigned long lastFirebaseMillis = 0;
const unsigned long FIREBASE_INTERVAL_MS = 30000; // send every 30 sec
unsigned long lastControlsPoll = 0;
const unsigned long CONTROLS_POLL_MS = 2000; // poll controls every 2 sec

// ------------------ SETUP ------------------
void setup() {
  Serial.begin(115200);
  delay(100);

  // ---------- WiFi ----------
  Serial.print("Connecting to WiFi ");
  Serial.print(ssid);
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - start > 20000) { // 20s timeout
      Serial.println("\nWiFi connect timeout - will continue and retry in loop");
      break;
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi not connected yet.");
  }

  // ---------- Firebase init ----------
  config.database_url = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  Serial.println("Firebase initialized (attempt).");

  // ---------- OLED init ----------
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    while (true) delay(10);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Smart Office + Relays");
  display.println("Initializing...");
  display.display();

  // ---------- ADC buffer ----------
  for (int i = 0; i < NUM_SAMPLES; i++) samples[i] = 0;
  analogSetPinAttenuation(MQ_PIN, ADC_11db);

  // ---------- Relay pins ----------
  pinMode(PURIFIER_RELAY_PIN, OUTPUT);
  pinMode(FAN_RELAY_PIN, OUTPUT);
  pinMode(LIGHT_RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(RELAY_LEFT, OUTPUT);
  pinMode(RELAY_RIGHT, OUTPUT);

  // Initialize all relays OFF (adapted to your reversed logic)
  digitalWrite(PURIFIER_RELAY_PIN, LOW); // purifier OFF
  digitalWrite(FAN_RELAY_PIN, HIGH);     // fan OFF (active LOW)
  digitalWrite(LIGHT_RELAY_PIN, HIGH);   // light OFF (active LOW)
  digitalWrite(LED_PIN, LOW);            // LED OFF
  digitalWrite(RELAY_LEFT, HIGH);        // left OFF (active LOW)
  digitalWrite(RELAY_RIGHT, HIGH);       // right OFF (active LOW)

  // ---------- Sensors ----------
  dht.begin();
  pinMode(PIR_PIN, INPUT);

  delay(1000);
  Serial.println("✅ Smart Office Ready");
}

// ------------------ AIR QUALITY STATUS ------------------
String getAQStatus(int val) {
  if (val < AQ_THRESHOLD * 0.75) return "GOOD";
  if (val < AQ_THRESHOLD * 1.1) return "MODERATE";
  return "POOR";
}

// ------------------ MAIN LOOP ------------------
unsigned long lastMillis = 0;
void loop() {
  handleSerialRelayControl();  // manual relay input via Serial

  // sample interval
  if (millis() - lastMillis >= SAMPLE_INTERVAL_MS) {
    lastMillis = millis();

    // ---------- AIR QUALITY ----------
    int raw = analogRead(MQ_PIN);
    samples[sampleIndex++] = raw;
    if (sampleIndex >= NUM_SAMPLES) sampleIndex = 0;
    long sum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) sum += samples[i];
    int avg = sum / NUM_SAMPLES;
    String AQstatus = getAQStatus(avg);
    digitalWrite(PURIFIER_RELAY_PIN, (avg >= AQ_THRESHOLD) ? HIGH : LOW); // Active HIGH

    // ---------- TEMPERATURE (Fan Active LOW) ----------
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();

    if (!isnan(temperature) && !isnan(humidity)) {
      if (!fanState && temperature > TEMP_THRESHOLD) {
        fanState = true;
        digitalWrite(FAN_RELAY_PIN, LOW); // ON
        Serial.println("Fan ON");
      } else if (fanState && temperature < (TEMP_THRESHOLD - HYSTERESIS)) {
        fanState = false;
        digitalWrite(FAN_RELAY_PIN, HIGH); // OFF
        Serial.println("Fan OFF");
      }
    } else {
      Serial.println("DHT read failed.");
    }

    // ---------- PIR LIGHT (Active LOW) ----------
    pirState = digitalRead(PIR_PIN);
    digitalWrite(LIGHT_RELAY_PIN, pirState ? LOW : HIGH); // ON when motion

    // ---------- LDR LED ----------
    int ldrValue = analogRead(LDR_PIN);
    if (ldrValue > LDR_THRESHOLD) {
      ledState = true;
      digitalWrite(LED_PIN, HIGH);
    } else {
      ledState = false;
      digitalWrite(LED_PIN, LOW);
    }

    // ---------- OLED ----------
    if (millis() - lastScreenMillis > SCREEN_INTERVAL_MS) {
      currentScreen = (currentScreen + 1) % NUM_SCREENS;
      lastScreenMillis = millis();
    }
    updateOLED(avg, AQstatus, temperature, humidity, ldrValue);

    // ---------- Serial log ----------
    Serial.print("[AQ]="); Serial.print(avg);
    Serial.print(" [T]="); Serial.print(temperature);
    Serial.print(" [H]="); Serial.print(humidity);
    Serial.print(" Fan="); Serial.print(fanState ? "ON" : "OFF");
    Serial.print(" PIR="); Serial.print(pirState ? "ON" : "OFF");
    Serial.print(" LED="); Serial.print(ledState ? "ON" : "OFF");
    Serial.print(" LDR="); Serial.print(ldrValue);
    Serial.print(" LRelay="); Serial.print(digitalRead(RELAY_LEFT)==LOW ? "ON" : "OFF");
    Serial.print(" RRelay="); Serial.println(digitalRead(RELAY_RIGHT)==LOW ? "ON" : "OFF");

    // ---------- FIREBASE writes (time-series + current state) ----------
    if (millis() - lastFirebaseMillis >= FIREBASE_INTERVAL_MS) {
      lastFirebaseMillis = millis();
      sendToFirebase(avg, temperature, humidity, fanState, pirState, ledState, ldrValue);
    }
  }

  // ---------- Poll controls (every CONTROLS_POLL_MS) ----------
  if (millis() - lastControlsPoll >= CONTROLS_POLL_MS) {
    lastControlsPoll = millis();
    pollControlsAndApply();
  }

  delay(5);
}

// ------------------ OLED UPDATE ------------------
void updateOLED(int aq, String aqStatus, float temp, float hum, int ldr) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  switch (currentScreen) {
    case 0:
      display.fillRect(0, 0, 128, 14, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(5, 3);
      display.print("AIR QUALITY");
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 20);
      display.setTextSize(2);
      display.print(aq);
      display.setCursor(0, 42);
      display.setTextSize(1);
      display.print("Status: "); display.println(aqStatus);
      break;
    case 1:
      display.fillRect(0, 0, 128, 14, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(5, 3);
      display.print("TEMP & FAN");
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 20);
      display.print("Temp: "); display.print(temp, 1); display.println("C");
      display.setCursor(0, 35);
      display.print("Hum: "); display.print(hum, 0); display.println("%");
      display.setCursor(0, 50);
      display.print("Fan: "); display.println(fanState ? "ON" : "OFF");
      break;
    case 2:
      display.fillRect(0, 0, 128, 14, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(5, 3);
      display.print("LIGHT STATUS");
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 20);
      display.print("PIR Light: "); display.println(pirState ? "ON" : "OFF");
      display.setCursor(0, 35);
      display.print("LED: "); display.println(ledState ? "ON" : "OFF");
      display.setCursor(0, 50);
      display.print("LDR: "); display.println(ldr);
      break;
  }
  display.display();
}

// ------------------ SERIAL RELAY CONTROL ------------------
void handleSerialRelayControl() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n') {
      if (dataSerial.length() >= 2) {
        char leftState = dataSerial[0];
        char rightState = dataSerial[1];
        Serial.print("Serial cmd: ");
        Serial.println(dataSerial);
        digitalWrite(RELAY_LEFT, (leftState == '1') ? LOW : HIGH);
        digitalWrite(RELAY_RIGHT, (rightState == '1') ? LOW : HIGH);
      }
      dataSerial = "";
    } else if (ch >= '0' && ch <= '9') {
      dataSerial += ch;
    }
  }
}

// ------------------ FIREBASE: send snapshot + timeseries ------------------
void sendToFirebase(int airQualityAvg, float temperature, float humidity, bool fanState,
                    bool pirState, bool ledState, int ldrValue) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected; skipping DB write.");
    return;
  }
  if (!Firebase.ready()) {
    Serial.println("Firebase not ready; skipping DB write.");
    return;
  }

  unsigned long ts = millis();
  String docId = "sensor_" + String(ts);
  String timeSeriesPath = "/sensor_readings/" + docId;
  String statePath = "/state"; // latest snapshot

  FirebaseJson json;
  json.set("air_quality", airQualityAvg);
  json.set("air_quality_status", getAQStatus(airQualityAvg));
  json.set("temperature", temperature);
  json.set("humidity", humidity);
  json.set("fan_on", fanState ? 1 : 0);
  json.set("pir_motion", pirState ? 1 : 0);
  json.set("led_on", ledState ? 1 : 0);
  json.set("ldr_value", ldrValue);
  json.set("left_relay", digitalRead(RELAY_LEFT)==LOW ? 1 : 0);
  json.set("right_relay", digitalRead(RELAY_RIGHT)==LOW ? 1 : 0);
  json.set("device_id", "esp32_001");
  json.set("location", "room_1");
  json.set("timestamp_ms", ts);

  // write time-series
  if (Firebase.setJSON(fbdo, timeSeriesPath.c_str(), json)) {
    Serial.println("Time-series written: " + timeSeriesPath);
  } else {
    Serial.println("Failed TS write: " + fbdo.errorReason());
  }

  // write snapshot (latest)
  if (Firebase.setJSON(fbdo, statePath.c_str(), json)) {
    Serial.println("State snapshot updated.");
  } else {
    Serial.println("Failed state write: " + fbdo.errorReason());
  }
}

// ------------------ POLL /controls and apply ------------------
void pollControlsAndApply() {
  if (WiFi.status() != WL_CONNECTED || !Firebase.ready()) return;

  // Controls expected at:
  // /controls/fan (0/1)
  // /controls/purifier
  // /controls/light
  // /controls/led
  // /controls/left_relay
  // /controls/right_relay
  // If a node doesn't exist or returns error, we ignore it.

  String base = "/controls/";

  // Helper lambda for reading int safely
  auto readIntNode = [&](const String &key, int &outVal) -> bool {
    String path = base + key;
    if (Firebase.getInt(fbdo, path.c_str())) {
      outVal = fbdo.intData();
      return true;
    }
    return false;
  };

  int val;

  if (readIntNode("fan", val)) {
    bool desired = (val != 0);
    if (desired != fanState) {
      fanState = desired;
      digitalWrite(FAN_RELAY_PIN, fanState ? LOW : HIGH); // active LOW
      Serial.printf("Control: fan -> %s\n", fanState ? "ON" : "OFF");
      // write back state snapshot
      Firebase.setInt(fbdo, "/state/fan_on", fanState ? 1 : 0);
    }
  }

  if (readIntNode("purifier", val)) {
    bool desired = (val != 0);
    digitalWrite(PURIFIER_RELAY_PIN, desired ? HIGH : LOW); // purifier active HIGH
    Serial.printf("Control: purifier -> %s\n", desired ? "ON" : "OFF");
    Firebase.setInt(fbdo, "/state/purifier_on", desired ? 1 : 0);
  }

  if (readIntNode("light", val)) {
    bool desired = (val != 0);
    // LIGHT_RELAY_PIN is active LOW when pir motion -> keep consistent
    digitalWrite(LIGHT_RELAY_PIN, desired ? LOW : HIGH);
    Serial.printf("Control: light -> %s\n", desired ? "ON" : "OFF");
    Firebase.setInt(fbdo, "/state/light_on", desired ? 1 : 0);
  }

  if (readIntNode("led", val)) {
    bool desired = (val != 0);
    digitalWrite(LED_PIN, desired ? HIGH : LOW);
    Serial.printf("Control: led -> %s\n", desired ? "ON" : "OFF");
    Firebase.setInt(fbdo, "/state/led_on", desired ? 1 : 0);
  }

  if (readIntNode("left_relay", val)) {
    bool desired = (val != 0);
    digitalWrite(RELAY_LEFT, desired ? LOW : HIGH); // active LOW
    Serial.printf("Control: left_relay -> %s\n", desired ? "ON" : "OFF");
    Firebase.setInt(fbdo, "/state/left_relay", desired ? 1 : 0);
  }

  if (readIntNode("right_relay", val)) {
    bool desired = (val != 0);
    digitalWrite(RELAY_RIGHT, desired ? LOW : HIGH);
    Serial.printf("Control: right_relay -> %s\n", desired ? "ON" : "OFF");
    Firebase.setInt(fbdo, "/state/right_relay", desired ? 1 : 0);
  }
}