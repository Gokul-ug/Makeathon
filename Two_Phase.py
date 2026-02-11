from ultralytics import YOLO
import cv2
import numpy as np
import serial
import time

# Load YOLOv8 model
model = YOLO("yolov8n.pt")  # or yolov8s.pt for better accuracy

# Serial setup (adjust COM port as needed)
ser = serial.Serial('COM5', 115200)
time.sleep(2)

# ----------------------------
# Open external webcam (index 1)
# ----------------------------
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)  # Use DirectShow for USB webcams
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print("❌ Could not open webcam.")
    exit()

original_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
original_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)
print(f"✅ Webcam opened: {original_width}x{original_height} @ {fps} FPS")

# Resize target for speed
target_size = (640, 360)

# Define polygon points (original resolution)
P1 = np.array([497, 375])
P2 = np.array([1076, 417])
P3 = np.array([1247, 623])
P4 = np.array([124, 420])

# Scale to resized frame
scale_x = target_size[0] / original_width
scale_y = target_size[1] / original_height
P1 = (P1 * [scale_x, scale_y]).astype(int)
P2 = (P2 * [scale_x, scale_y]).astype(int)
P3 = (P3 * [scale_x, scale_y]).astype(int)
P4 = (P4 * [scale_x, scale_y]).astype(int)

# Midpoints for dividing polygon vertically (Left/Right)
top_mid = ((P1 + P2) / 2).astype(int)
bottom_mid = ((P4 + P3) / 2).astype(int)

regions = {
    "Q1_Left":  [P1, top_mid, bottom_mid, P4],
    "Q2_Right": [top_mid, P2, P3, bottom_mid]
}
region_polygons = {k: np.array(v, dtype=np.int32) for k, v in regions.items()}

frame_skip = 2
frame_count = 0

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("⚠️ Frame not received. Exiting...")
        break

    frame_count += 1
    if frame_count % frame_skip != 0:
        continue

    resized = cv2.resize(frame, target_size)
    results = model(resized, conf=0.25, iou=0.3, agnostic_nms=True, verbose=False)
    boxes = results[0].boxes

    region_counts = {key: 0 for key in regions.keys()}

    for poly in region_polygons.values():
        cv2.polylines(resized, [poly], isClosed=True, color=(255, 255, 0), thickness=2)

    if boxes is not None:
        for box, cls in zip(boxes.xyxy, boxes.cls):
            if int(cls) == 0:  # person
                x1, y1, x2, y2 = map(int, box)
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                found = False

                for region_name, polygon in region_polygons.items():
                    if cv2.pointPolygonTest(polygon, (cx, cy), False) >= 0:
                        region_counts[region_name] += 1
                        color = (0, 255, 0)
                        found = True
                        break

                cv2.rectangle(resized, (x1, y1), (x2, y2), color if found else (0, 0, 255), 2)
                cv2.circle(resized, (cx, cy), 4, color if found else (0, 0, 255), -1)

    binary_str = ''.join(['1' if region_counts[key] > 0 else '0' for key in regions.keys()])
    print(f"[DEBUG] Region counts: {[region_counts[k] for k in regions.keys()]} → Sending: {binary_str}")
    ser.write((binary_str + "\n").encode())

    y_offset = 30
    for i, (region, count) in enumerate(region_counts.items()):
        cv2.putText(resized, f"{region}: {count}", (10, y_offset + 30 * i),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    cv2.imshow("2-Quadrant (Left/Right) Detection", resized)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
ser.close()





"""/*
  Smart Office + Serial Relay Control (Reversed Fan & PIR Logic)
  ESP32 + MQ135 + DHT11 + PIR + LDR + OLED + 5 Relays + LED
  Author: Arun Roshan Tech / ChatGPT
  Date: 2025-10-12
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "DHT.h"

// ------------------ OLED CONFIG ------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define OLED_SDA 21
#define OLED_SCL 22

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
String data = "";

// ------------------ OLED SCREEN SHUFFLE ------------------
unsigned long lastScreenMillis = 0;
const int SCREEN_INTERVAL_MS = 4000;
int currentScreen = 0;
const int NUM_SCREENS = 3;

// ------------------ SETUP ------------------
void setup() {
  Serial.begin(115200);
  delay(100);

  // OLED init
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

  // ADC buffer
  for (int i = 0; i < NUM_SAMPLES; i++) samples[i] = 0;
  analogSetPinAttenuation(MQ_PIN, ADC_11db);

  // Relay pins
  pinMode(PURIFIER_RELAY_PIN, OUTPUT);
  pinMode(FAN_RELAY_PIN, OUTPUT);
  pinMode(LIGHT_RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(RELAY_LEFT, OUTPUT);
  pinMode(RELAY_RIGHT, OUTPUT);

  // Initialize all relays OFF (active LOW for Fan & PIR)
  digitalWrite(PURIFIER_RELAY_PIN, LOW); // Active HIGH (air purifier)
  digitalWrite(FAN_RELAY_PIN, HIGH);     // Reversed: HIGH = OFF
  digitalWrite(LIGHT_RELAY_PIN, HIGH);   // Reversed: HIGH = OFF
  digitalWrite(LED_PIN, LOW);            // Normal
  digitalWrite(RELAY_LEFT, HIGH);        // Active LOW
  digitalWrite(RELAY_RIGHT, HIGH);       // Active LOW

  // Sensor init
  dht.begin();
  pinMode(PIR_PIN, INPUT);

  delay(1000);
  Serial.println("✅ Smart Office + Serial Relay Ready (Reversed Fan & PIR Logic)");
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
  handleSerialRelayControl();  // Manual relay input handler

  if (millis() - lastMillis < SAMPLE_INTERVAL_MS) return;
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
      digitalWrite(FAN_RELAY_PIN, LOW); // ON (active LOW)
      Serial.println("Fan ON - Overheat detected!");
    } else if (fanState && temperature < (TEMP_THRESHOLD - HYSTERESIS)) {
      fanState = false;
      digitalWrite(FAN_RELAY_PIN, HIGH); // OFF (active LOW)
      Serial.println("Fan OFF - Temp normal.");
    }
  }

  // ---------- PIR LIGHT (Active LOW) ----------
  pirState = digitalRead(PIR_PIN);
  digitalWrite(LIGHT_RELAY_PIN, pirState ? LOW : HIGH); // ON when motion detected

  // ---------- LDR LED ----------
  int ldrValue = analogRead(LDR_PIN);
  if (ldrValue > LDR_THRESHOLD) {
    ledState = true;
    digitalWrite(LED_PIN, HIGH); // Normal logic
  } else {
    ledState = false;
    digitalWrite(LED_PIN, LOW);
  }

  // ---------- OLED SCREEN ----------
  if (millis() - lastScreenMillis > SCREEN_INTERVAL_MS) {
    currentScreen = (currentScreen + 1) % NUM_SCREENS;
    lastScreenMillis = millis();
  }

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
      display.print(avg);
      display.setCursor(0, 42);
      display.setTextSize(1);
      display.print("Status: "); display.println(AQstatus);
      break;

    case 1:
      display.fillRect(0, 0, 128, 14, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(5, 3);
      display.print("TEMP & FAN");
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 20);
      display.print("Temp: "); display.print(temperature, 1); display.println("C");
      display.setCursor(0, 35);
      display.print("Hum: "); display.print(humidity, 0); display.println("%");
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
      display.print("LDR: "); display.println(ldrValue);
      break;
  }

  display.display();

  // ---------- SERIAL LOGGING ----------
  Serial.print("[AQ]="); Serial.print(avg);
  Serial.print(" | [TEMP]="); Serial.print(temperature);
  Serial.print(" | [HUM]="); Serial.print(humidity);
  Serial.print(" | Fan="); Serial.print(fanState ? "ON" : "OFF");
  Serial.print(" | PIR="); Serial.print(pirState ? "ON" : "OFF");
  Serial.print(" | LED="); Serial.print(ledState ? "ON" : "OFF");
  Serial.print(" | LDR="); Serial.print(ldrValue);
  Serial.print(" | L/R Relay="); 
  Serial.print(digitalRead(RELAY_LEFT)==LOW ? "L:ON " : "L:OFF ");
  Serial.println(digitalRead(RELAY_RIGHT)==LOW ? "R:ON" : "R:OFF");
}

// ------------------ SERIAL RELAY CONTROL ------------------
void handleSerialRelayControl() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n') {
      if (data.length() == 2) {
        char leftState = data[0];
        char rightState = data[1];

        Serial.print("📩 Received: ");
        Serial.println(data);

        digitalWrite(RELAY_LEFT, (leftState == '1') ? LOW : HIGH);
        digitalWrite(RELAY_RIGHT, (rightState == '1') ? LOW : HIGH);
      }
      data = "";
    } else if (ch >= '0' && ch <= '9') {
      data += ch;
    }
  }
}
"""

"""/*
  Smart Office + Serial Relay Control
  ESP32 + MQ135 + DHT11 + PIR + LDR + OLED + 5 Relays + LED
  Author: Arun Roshan Tech / ChatGPT
  Date: 2025-10-12
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "DHT.h"

// ------------------ OLED CONFIG ------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define OLED_SDA 21
#define OLED_SCL 22

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
bool fanState = false;

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
String data = "";

// ------------------ OLED SCREEN SHUFFLE ------------------
unsigned long lastScreenMillis = 0;
const int SCREEN_INTERVAL_MS = 4000;
int currentScreen = 0;
const int NUM_SCREENS = 3;

// ------------------ SETUP ------------------
void setup() {
  Serial.begin(115200);
  delay(100);

  // OLED init
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

  // ADC buffer
  for (int i = 0; i < NUM_SAMPLES; i++) samples[i] = 0;
  analogSetPinAttenuation(MQ_PIN, ADC_11db);

  // Relay pins
  pinMode(PURIFIER_RELAY_PIN, OUTPUT);
  pinMode(FAN_RELAY_PIN, OUTPUT);
  pinMode(LIGHT_RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(RELAY_LEFT, OUTPUT);
  pinMode(RELAY_RIGHT, OUTPUT);

  // Initialize all relays OFF (active LOW)
  digitalWrite(PURIFIER_RELAY_PIN, LOW);
  digitalWrite(FAN_RELAY_PIN, LOW);
  digitalWrite(LIGHT_RELAY_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(RELAY_LEFT, HIGH);
  digitalWrite(RELAY_RIGHT, HIGH);

  // Sensor init
  dht.begin();
  pinMode(PIR_PIN, INPUT);

  delay(1000);
  Serial.println("✅ Smart Office + Serial Relay Ready");
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
  handleSerialRelayControl();  // Manual relay input handler

  if (millis() - lastMillis < SAMPLE_INTERVAL_MS) return;
  lastMillis = millis();

  // ---------- AIR QUALITY ----------
  int raw = analogRead(MQ_PIN);
  samples[sampleIndex++] = raw;
  if (sampleIndex >= NUM_SAMPLES) sampleIndex = 0;

  long sum = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) sum += samples[i];
  int avg = sum / NUM_SAMPLES;
  String AQstatus = getAQStatus(avg);
  digitalWrite(PURIFIER_RELAY_PIN, (avg >= AQ_THRESHOLD) ? HIGH : LOW);

  // ---------- TEMPERATURE ----------
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  if (!isnan(temperature) && !isnan(humidity)) {
    if (!fanState && temperature > TEMP_THRESHOLD) {
      fanState = true;
      digitalWrite(FAN_RELAY_PIN, HIGH);
      Serial.println("Fan ON - Overheat detected!");
    } else if (fanState && temperature < (TEMP_THRESHOLD - HYSTERESIS)) {
      fanState = false;
      digitalWrite(FAN_RELAY_PIN, LOW);
      Serial.println("Fan OFF - Temp normal.");
    }
  }

  // ---------- PIR LIGHT ----------
  pirState = digitalRead(PIR_PIN);
  digitalWrite(LIGHT_RELAY_PIN, pirState ? HIGH : LOW);

  // ---------- LDR LED ----------
  int ldrValue = analogRead(LDR_PIN);
  if (ldrValue > LDR_THRESHOLD) {
    ledState = true;
    digitalWrite(LED_PIN, HIGH);
  } else {
    ledState = false;
    digitalWrite(LED_PIN, LOW);
  }

  // ---------- OLED SCREEN ----------
  if (millis() - lastScreenMillis > SCREEN_INTERVAL_MS) {
    currentScreen = (currentScreen + 1) % NUM_SCREENS;
    lastScreenMillis = millis();
  }

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
      display.print(avg);
      display.setCursor(0, 42);
      display.setTextSize(1);
      display.print("Status: "); display.println(AQstatus);
      break;

    case 1:
      display.fillRect(0, 0, 128, 14, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(5, 3);
      display.print("TEMP & FAN");
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 20);
      display.print("Temp: "); display.print(temperature, 1); display.println("C");
      display.setCursor(0, 35);
      display.print("Hum: "); display.print(humidity, 0); display.println("%");
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
      display.print("LDR: "); display.println(ldrValue);
      break;
  }

  display.display();

  // ---------- SERIAL LOGGING ----------
  Serial.print("[AQ]="); Serial.print(avg);
  Serial.print(" | [TEMP]="); Serial.print(temperature);
  Serial.print(" | [HUM]="); Serial.print(humidity);
  Serial.print(" | Fan="); Serial.print(fanState ? "ON" : "OFF");
  Serial.print(" | PIR="); Serial.print(pirState ? "ON" : "OFF");
  Serial.print(" | LED="); Serial.print(ledState ? "ON" : "OFF");
  Serial.print(" | LDR="); Serial.print(ldrValue);
  Serial.print(" | L/R Relay="); 
  Serial.print(digitalRead(RELAY_LEFT)==LOW ? "L:ON " : "L:OFF ");
  Serial.println(digitalRead(RELAY_RIGHT)==LOW ? "R:ON" : "R:OFF");
}

// ------------------ SERIAL RELAY CONTROL ------------------
void handleSerialRelayControl() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n') {
      if (data.length() == 2) {
        char leftState = data[0];
        char rightState = data[1];

        Serial.print("📩 Received: ");
        Serial.println(data);

        digitalWrite(RELAY_LEFT, (leftState == '1') ? LOW : HIGH);
        digitalWrite(RELAY_RIGHT, (rightState == '1') ? LOW : HIGH);
      }
      data = "";
    } else if (ch >= '0' && ch <= '9') {
      data += ch;
    }
  }
}
"""