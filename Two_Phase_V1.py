from ultralytics import YOLO
import cv2
import numpy as np
import serial, time

# -------------------- MODEL SETUP --------------------
model = YOLO("yolov8n.pt")
print("✅ YOLOv8 model loaded")

# -------------------- SERIAL SETUP --------------------
try:
    ser = serial.Serial('COM5', 115200, timeout=1)
    time.sleep(2)
    print("✅ Connected to ESP32")
except serial.SerialException:
    ser = None
    print("⚠️ ESP32 not connected (visual-only mode)")

# -------------------- CAMERA SETUP --------------------
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print("❌ Could not open webcam")
    exit()

# -------------------- MANUAL REGION SELECTION --------------------
clicked_points = []

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and len(clicked_points) < 4:
        clicked_points.append((x, y))
        print(f"✅ Point {len(clicked_points)}: ({x}, {y})")

cv2.namedWindow("Setup")
cv2.setMouseCallback("Setup", mouse_callback)

print("🟡 Click 4 points in order (top-left → top-right → bottom-right → bottom-left)")

while len(clicked_points) < 4:
    ret, frame = cap.read()
    if not ret:
        continue
    temp = frame.copy()
    for p in clicked_points:
        cv2.circle(temp, p, 6, (0, 255, 255), -1)
    cv2.imshow("Setup", temp)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        exit()

cv2.destroyWindow("Setup")

# Convert to NumPy arrays
P1, P2, P3, P4 = map(np.array, clicked_points)

# -------------------- VERTICAL SPLIT --------------------
# Find midpoints of top and bottom edges
top_mid = ((P1 + P2) / 2).astype(int)
bottom_mid = ((P4 + P3) / 2).astype(int)

# Create Left and Right region polygons
left_region = np.array([P1, top_mid, bottom_mid, P4], dtype=np.int32)
right_region = np.array([top_mid, P2, P3, bottom_mid], dtype=np.int32)  

regions = {"Left": left_region, "Right": right_region}
print("✅ Region vertically divided into Left and Right halves.")

# -------------------- DETECTION LOOP --------------------
while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame, conf=0.4, verbose=False)
    boxes = results[0].boxes

    region_counts = {"Left": 0, "Right": 0}

    # Draw base regions
    overlay = frame.copy()
    cv2.polylines(overlay, [left_region], True, (0, 255, 0), 2)
    cv2.polylines(overlay, [right_region], True, (0, 0, 255), 2)
    cv2.line(overlay, tuple(top_mid), tuple(bottom_mid), (255, 255, 0), 2)  # center vertical divider

    # Process detections
    if boxes is not None:
        for box, cls in zip(boxes.xyxy, boxes.cls):
            if int(cls) == 0:  # person class
                x1, y1, x2, y2 = map(int, box)
                cx, cy = (x1 + x2)//2, (y1 + y2)//2

                # Check if point is inside left/right region
                for region_name, poly in regions.items():
                    if cv2.pointPolygonTest(poly, (cx, cy), False) >= 0:
                        region_counts[region_name] += 1
                        color = (0, 255, 0) if region_name == "Left" else (0, 0, 255)
                        cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2)
                        cv2.circle(overlay, (cx, cy), 5, color, -1)
                        cv2.putText(overlay, region_name, (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                        break

    # Display counts
    cv2.putText(overlay, f"Left: {region_counts['Left']}", (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(overlay, f"Right: {region_counts['Right']}", (30, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    # Send binary data to ESP32
    binary_str = f"{int(region_counts['Left'] > 0)}{int(region_counts['Right'] > 0)}"
    if ser:
        ser.write((binary_str + "\n").encode())
    print(f"[DEBUG] Sent: {binary_str} | Counts: {region_counts}")

    # Display
    cv2.imshow("Vertical Split Region Detection", overlay)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# -------------------- CLEANUP --------------------
cap.release()
cv2.destroyAllWindows()
if ser:
    ser.close()