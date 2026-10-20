import cv2
import numpy as np
from pathlib import Path

OUT = Path(__file__).parent
IMG_DIR = OUT / "checkerboard_images"
IMG_DIR.mkdir(exist_ok=True)

CAMERA_ID = 2

# Jumlah inner corners: kolom, baris
BOARD_SIZE = (8, 5)

SQUARE_SIZE_MM = 30

cap = cv2.VideoCapture(CAMERA_ID)

if not cap.isOpened():
    raise RuntimeError("Kamera tidak dapat dibuka")

obj_template = np.zeros(
    (BOARD_SIZE[0] * BOARD_SIZE[1], 3),
    np.float32
)
obj_template[:, :2] = np.mgrid[
    0:BOARD_SIZE[0], 0:BOARD_SIZE[1]
].T.reshape(-1, 2) * SQUARE_SIZE_MM

obj_points = []
img_points = []
image_size = None
saved_count = 0

criteria = (
    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
    30, 0.001
)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Gagal membaca kamera")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    image_size = (gray.shape[1], gray.shape[0])

    found, corners = cv2.findChessboardCorners(
        gray, BOARD_SIZE, None
    )

    display = frame.copy()

    if found:
        refined = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1), criteria
        )
        cv2.drawChessboardCorners(
            display, BOARD_SIZE, refined, found
        )
        cv2.putText(
            display, "Pattern detected - press s to save",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (0, 255, 0), 2
        )

    cv2.putText(
        display,
        f"Saved: {saved_count} | s=save c=calibrate q=quit",
        (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
        0.6, (0, 255, 255), 2
    )

    cv2.imshow("Camera Calibration", display)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("s"):
        if found:
            path = IMG_DIR / f"calib_{saved_count:03d}.png"
            success = cv2.imwrite(str(path), frame)
            
            if success:
                obj_points.append(obj_template.copy())
                img_points.append(refined.copy())
                saved_count += 1
                print("Disimpan:", path)
            else:
                print("Gagal menyimpan gambar ke disk!")
        else:
            print("Papan catur (chessboard) tidak terdeteksi pada frame ini!")

    elif key == ord("c"):
        if saved_count < 10:
            print("Ambil minimal 10 gambar; 15-25 lebih disarankan")
            continue

        rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
            obj_points,
            img_points,
            image_size,
            None,
            None
        )

        np.savez(
            OUT / "camera_calib.npz",
            K=K,
            dist=dist,
            image_width=image_size[0],
            image_height=image_size[1],
            rms=rms
        )

        print("\nKalibrasi selesai")
        print("RMS reprojection error:", rms)
        print("Camera matrix:\n", K)
        print("Distortion coefficients:\n", dist)
        print("Disimpan ke camera_calib.npz")

    elif key == ord("q"):
        if key == ord("q"):
            break

cap.release()
cv2.destroyAllWindows()