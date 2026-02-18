#include <WiFi.h>
#include <FirebaseESP32.h>
#include "DHT.h"

// =========== WiFi & Firebase ===========
const char* ssid = "mdkaif";
const char* password = "123456789";

#define FIREBASE_HOST "iotmakethon123-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "EkPsSjXqlDLwqG8BJRg89YyO5eZUNSHNG6168tED"

FirebaseData fbdo;
FirebaseConfig config;
FirebaseAuth auth;

// =========== Pins ===========
#define MQ_PIN 34
#define DHTPIN 27
#define DHTTYPE DHT11
#define PIR_PIN 14
#define LDR_PIN 35
#define FAN 25
#define PURIFIER 26
#define LIGHT 13
#define LED 15
#define LEFT 2
#define RIGHT 4

DHT dht(DHTPIN, DHTTYPE);

// =========== Variables ===========
String mode = "auto";
bool fan_on, purifier_on, light_on, led_on, left_on, right_on;
unsigned long lastSend = 0;

// =========== Setup ===========
void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi connected.");

  config.database_url = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&config, &auth);

  pinMode(FAN, OUTPUT);
  pinMode(PURIFIER, OUTPUT);
  pinMode(LIGHT, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(LEFT, OUTPUT);
  pinMode(RIGHT, OUTPUT);

  dht.begin();
  Serial.println("Smart Office Ready!");
}

// =========== Helper ===========
void updateFirebaseState() {
  FirebaseJson json;
  json.set("fan_on", fan_on);
  json.set("purifier_on", purifier_on);
  json.set("light_on", light_on);
  json.set("led_on", led_on);
  json.set("left_relay", left_on);
  json.set("right_relay", right_on);
  json.set("temperature", dht.readTemperature());
  json.set("humidity", dht.readHumidity());
  json.set("air_quality", analogRead(MQ_PIN));
  json.set("ldr_value", analogRead(LDR_PIN));
  json.set("pir_motion", digitalRead(PIR_PIN));
  Firebase.setJSON(fbdo, "/state", json);
}

void setAllOff() {
  fan_on = purifier_on = light_on = led_on = left_on = right_on = false;
  digitalWrite(FAN, HIGH);
  digitalWrite(PURIFIER, LOW);
  digitalWrite(LIGHT, HIGH);
  digitalWrite(LED, LOW);
  digitalWrite(LEFT, HIGH);
  digitalWrite(RIGHT, HIGH);
  updateFirebaseState();
}

// =========== Loop ===========
void loop() {
  Firebase.getString(fbdo, "/mode");
  if (fbdo.dataType() == "string") mode = fbdo.stringData();

  if (mode == "off") {
    setAllOff();
  }
  else if (mode == "auto") {
    float temp = dht.readTemperature();
    int mq = analogRead(MQ_PIN);
    int ldr = analogRead(LDR_PIN);
    bool pir = digitalRead(PIR_PIN);

    fan_on = temp > 30;
    purifier_on = mq > 700;
    light_on = pir;
    led_on = ldr > 1500;
    left_on = right_on = false;

    digitalWrite(FAN, fan_on ? LOW : HIGH);
    digitalWrite(PURIFIER, purifier_on ? HIGH : LOW);
    digitalWrite(LIGHT, light_on ? LOW : HIGH);
    digitalWrite(LED, led_on ? HIGH : LOW);
  }
  else if (mode == "manual") {
    int val;
    if (Firebase.getInt(fbdo, "/controls/fan")) fan_on = fbdo.intData() % 2;
    if (Firebase.getInt(fbdo, "/controls/purifier")) purifier_on = fbdo.intData() % 2;
    if (Firebase.getInt(fbdo, "/controls/light")) light_on = fbdo.intData() % 2;
    if (Firebase.getInt(fbdo, "/controls/led")) led_on = fbdo.intData() % 2;
    if (Firebase.getInt(fbdo, "/controls/left_relay")) left_on = fbdo.intData() % 2;
    if (Firebase.getInt(fbdo, "/controls/right_relay")) right_on = fbdo.intData() % 2;

    digitalWrite(FAN, fan_on ? LOW : HIGH);
    digitalWrite(PURIFIER, purifier_on ? HIGH : LOW);
    digitalWrite(LIGHT, light_on ? LOW : HIGH);
    digitalWrite(LED, led_on ? HIGH : LOW);
    digitalWrite(LEFT, left_on ? LOW : HIGH);
    digitalWrite(RIGHT, right_on ? LOW : HIGH);
  }

  if (millis() - lastSend > 2000) {
    updateFirebaseState();
    lastSend = millis();
  }
}