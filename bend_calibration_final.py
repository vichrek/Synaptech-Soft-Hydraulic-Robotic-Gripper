import sys
sys.path.append(r"C:\Users\16bam\AppData\Local\Programs\Python\Python312\Lib\site-packages")

import cv2
import numpy as np
import serial
import time
import matplotlib.pyplot as plt

SERIAL_PORT = "COM3"
BAUD_RATE = 9600
CAMERA_INDEX = 0
NUM_TESTS = 3

all_test_data = {}  # test_num -> {marker_id -> [(angle_change, bend1, bend2, bend3, force1, force2, force3)]}

arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
time.sleep(2)

cap = cv2.VideoCapture(CAMERA_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
if not cap.isOpened():
    arduino.close()
    raise RuntimeError("Could not open camera")

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

def marker_orientation_deg(marker4x2: np.ndarray) -> float:
    p0 = marker4x2[0]
    p1 = marker4x2[1]
    return float(np.degrees(np.arctan2(p1[1] - p0[1], p1[0] - p0[0])))

def run_test(test_num):
    print(f"\n--- TEST {test_num} ---")
    print("Press SPACE to start/pause recording, R to reset, Q to finish.")

    initial_angle = {}
    pairs = {}  # marker_id -> [(angle, b1, b2, b3, f1, f2, f3)]
    recording = False
    serial_ok = False

    latest = {
        "b1": None, "b2": None, "b3": None,
        "f1": None, "f2": None, "f3": None
    }

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # Read serial
            while arduino.in_waiting > 0:
                line = arduino.readline().decode("utf-8", errors="ignore").strip()
                if line.startswith("CALIB,"):
                    parts = line.split(",")
                    if len(parts) == 7:
                        try:
                            latest["b1"] = int(parts[1])
                            latest["b2"] = int(parts[2])
                            latest["b3"] = int(parts[3])
                            latest["f1"] = int(parts[4])
                            latest["f2"] = int(parts[5])
                            latest["f3"] = int(parts[6])
                            serial_ok = True
                        except:
                            pass

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            display = frame.copy()
            marker_ok = (ids is not None and len(ids) > 0)

            if marker_ok:
                cv2.aruco.drawDetectedMarkers(display, corners, ids)

                for i in range(len(ids)):
                    mid = int(ids[i][0])
                    marker = corners[i][0]
                    ang = marker_orientation_deg(marker)

                    if mid not in initial_angle:
                        initial_angle[mid] = ang
                        pairs.setdefault(mid, [])

                    rel = (ang - initial_angle[mid] + 180.0) % 360.0 - 180.0

                    if recording and all(v is not None for v in latest.values()):
                        pairs[mid].append((
                            rel,
                            latest["b1"], latest["b2"], latest["b3"],
                            latest["f1"], latest["f2"], latest["f3"]
                        ))

                    center = marker.mean(axis=0)
                    cx, cy = int(center[0]), int(center[1])
                    cv2.putText(display, f"ID {mid}: {rel:.1f} deg",
                                (cx - 50, cy - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

            # Overlay
            cv2.putText(display, f"TEST {test_num}/{NUM_TESTS}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 200, 0), 2)
            cv2.putText(display, f"Serial: {'OK' if serial_ok else 'NO DATA'}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                        (0, 255, 0) if serial_ok else (0, 0, 255), 2)
            cv2.putText(display, f"Marker: {'OK' if marker_ok else 'NOT DETECTED'}",
                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                        (0, 255, 0) if marker_ok else (0, 0, 255), 2)
            cv2.putText(display, f"Recording: {'YES' if recording else 'NO (SPACE)'}",
                        (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                        (0, 255, 0) if recording else (0, 165, 255), 2)

            if serial_ok:
                cv2.putText(display,
                            f"B1:{latest['b1']} B2:{latest['b2']} B3:{latest['b3']}",
                            (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(display,
                            f"F1:{latest['f1']} F2:{latest['f2']} F3:{latest['f3']}",
                            (10, 175), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 255), 2)

            total_pairs = sum(len(v) for v in pairs.values())
            cv2.putText(display, f"Pairs logged: {total_pairs}",
                        (10, 205), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)

            cv2.imshow("Tracker", display)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break
            elif key == ord(" "):
                recording = not recording
                print(f"Recording {'started' if recording else 'paused'}")
            elif key == ord("r"):
                initial_angle.clear()
                for k in pairs:
                    pairs[k].clear()
                print("Reset references + cleared data")

    finally:
        pass

    return pairs

try:
    for t in range(1, NUM_TESTS + 1):
        input(f"\nPress Enter in console when ready to start Test {t}...")
        all_test_data[t] = run_test(t)
        total = sum(len(v) for v in all_test_data[t].values())
        print(f"Test {t} complete — {total} pairs collected.")

finally:
    cap.release()
    cv2.destroyAllWindows()
    arduino.close()

# ── Plotting ───────────────────────────────────────────────
# One figure per test, with 3 subplots (one per finger: angle vs bend value)
finger_labels = ["Finger 1", "Finger 2", "Finger 3"]
bend_keys = [1, 2, 3]  # index into tuple: (angle, b1, b2, b3, f1, f2, f3)

for t in range(1, NUM_TESTS + 1):
    pairs = all_test_data.get(t, {})
    if not pairs or sum(len(v) for v in pairs.values()) == 0:
        print(f"Test {t}: no data to plot.")
        continue

    all_angles = []
    all_bends = []
    for mid, data in pairs.items():
        if len(data) < 5:
            continue
        for row in data:
            all_angles.append(row[0])
            all_bends.append(row[1])  # F1 bend — change to row[2] or row[3] for F2/F3

    if not all_angles:
        print(f"Test {t}: not enough data.")
        continue

    plt.figure(figsize=(7, 5))
    plt.scatter(all_angles, all_bends, s=10, alpha=0.6, color=f"C{t-1}")
    plt.xlabel("Angle change (deg)")
    plt.ylabel("Bend sensor ADC (0–1023)")
    plt.title(f"Test {t} — Finger {t} calibration")
    plt.grid(True, alpha=0.3)

plt.show()
