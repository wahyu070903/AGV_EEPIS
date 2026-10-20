
import cv2
import numpy as np
from pathlib import Path

# ==========================================
# 1. KONFIGURASI
# ==========================================

BASE = Path(__file__).parent

CAMERA_ID = 2

# Ukuran fisik marker (cm)
MARKER_SIZE = 10.0

# Resolusi bird's-eye view
OUT_W = 640
OUT_H = 480

# Koordinat fisik marker (cm)
# Posisi adalah sudut kiri-bawah marker
MARKER_POS = {
    0: (-28.0, 130.0),
    1: (194.0, 130.0),
    2: (194.0, 85.0),
    3: (-28.0, 85.0),
}

# Rentang area lantai yang ingin dipetakan (cm)
# Rasio area harus 4:3 agar sesuai dengan output 640x480.
X_MIN = -28.0
X_MAX = 204.0

# Lebar area = 232 cm
# Tinggi area = 174 cm
# Pilih area Y yang sesuai dengan kondisi fisik.
Y_MIN = 35.0
Y_MAX = 209.0

# Skala harus sama pada sumbu X dan Y
PX_PER_CM = OUT_W / (X_MAX - X_MIN)

assert abs(
    (Y_MAX - Y_MIN) * PX_PER_CM - OUT_H
) < 1e-6, "Rentang area tidak cocok dengan rasio output!"

# ==========================================
# 2. LOAD KALIBRASI INTRINSIK
# ==========================================

cal_path = BASE / "camera_calib.npz"

if not cal_path.exists():
    raise FileNotFoundError(
        f"File kalibrasi kamera tidak ditemukan: {cal_path}"
    )

cal = np.load(cal_path)
K = cal["K"]
dist = cal["dist"]

# ==========================================
# 3. SETUP ARUCO
# ==========================================

dictionary = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_4X4_50
)

if hasattr(cv2.aruco, "ArucoDetector"):
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(
        dictionary, parameters
    )
    use_new_api = True
else:
    parameters = cv2.aruco.DetectorParameters_create()
    detector = None
    use_new_api = False

# ==========================================
# 4. BUKA KAMERA
# ==========================================

cap = cv2.VideoCapture(CAMERA_ID)

if not cap.isOpened():
    raise RuntimeError(
        f"Kamera dengan ID {CAMERA_ID} gagal dibuka"
    )

H = None


# ==========================================
# 5. FUNGSI KOORDINAT
# ==========================================

def floor_to_pixel(X, Y):
    """Koordinat lantai (cm) -> koordinat output (pixel)."""
    u = (X - X_MIN) * PX_PER_CM
    v = (Y_MAX - Y) * PX_PER_CM

    return [u, v]


# ==========================================
# 6. LOOP KALIBRASI
# ==========================================

while True:
    ret, frame = cap.read()

    if not ret:
        print("Gagal membaca frame kamera")
        break

    # Koreksi distorsi kamera
    undistorted = cv2.undistort(frame, K, dist)

    # Deteksi ArUco
    if use_new_api:
        corners, ids, rejected = detector.detectMarkers(
            undistorted
        )
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(
            undistorted,
            dictionary,
            parameters=parameters
        )

    display = undistorted.copy()

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(
            display, corners, ids
        )

        id_to_corners = {
            int(marker_id): corner.reshape(4, 2)
            for marker_id, corner
            in zip(ids.flatten(), corners)
        }

        required_ids = set(MARKER_POS.keys())
        detected_ids = set(id_to_corners.keys())

        if required_ids.issubset(detected_ids):

            src_points = []
            dst_points = []

            # Susunan sudut ArUco:
            # kiri-atas, kanan-atas,
            # kanan-bawah, kiri-bawah
            for marker_id, (x, y) in MARKER_POS.items():

                image_corners = id_to_corners[marker_id]

                # Koordinat fisik empat sudut marker
                floor_corners = [
                    (x, y + MARKER_SIZE),
                    (x + MARKER_SIZE, y + MARKER_SIZE),
                    (x + MARKER_SIZE, y),
                    (x, y),
                ]

                src_points.extend(
                    image_corners.tolist()
                )

                dst_points.extend([
                    floor_to_pixel(X, Y)
                    for X, Y in floor_corners
                ])

            src = np.asarray(
                src_points, dtype=np.float32
            )
            dst = np.asarray(
                dst_points, dtype=np.float32
            )

            # Hitung homografi
            H_candidate, status = cv2.findHomography(
                src, dst, method=0
            )

            if H_candidate is not None:
                H = H_candidate

                cv2.putText(
                    display,
                    "CALIBRATION READY - PRESS S",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 0),
                    2
                )

                print(
                    "Marker lengkap:",
                    sorted(detected_ids)
                )

            else:
                cv2.putText(
                    display,
                    "HOMOGRAPHY FAILED",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 0, 255),
                    2
                )

        else:
            missing = required_ids - detected_ids

            cv2.putText(
                display,
                f"Missing markers: {sorted(missing)}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 0, 255),
                2
            )

    else:
        cv2.putText(
            display,
            "NO ARUCO DETECTED",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 255),
            2
        )

    # Tampilkan bird's-eye view
    if H is not None:
        bev = cv2.warpPerspective(
            undistorted,
            H,
            (OUT_W, OUT_H)
        )

        cv2.imshow("Bird's-Eye View 640x480", bev)

    cv2.imshow("Floor Homography Calibration", display)

    key = cv2.waitKey(1) & 0xFF

    # Simpan kalibrasi
    if key == ord("s") and H is not None:

        save_path = BASE / "floor_homography.npz"

        np.savez(
            save_path,
            H=H,
            out_w=OUT_W,
            out_h=OUT_H,
            px_per_cm=PX_PER_CM,
            x_min=X_MIN,
            x_max=X_MAX,
            y_min=Y_MIN,
            y_max=Y_MAX
        )

        print("\nKalibrasi berhasil disimpan!")
        print("File:", save_path)
        print("Homography matrix:\n", H)
        print("Resolusi output:", OUT_W, "x", OUT_H)
        print("Skala:", PX_PER_CM, "pixel/cm")

    elif key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
