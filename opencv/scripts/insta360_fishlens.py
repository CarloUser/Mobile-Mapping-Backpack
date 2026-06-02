import cv2
import numpy as np
import glob

# ===== SETTINGS =====
CHECKERBOARD = (9, 6)   # inner corners
SQUARE_SIZE = 0.025     # meters

# prepare object points
objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []
imgpoints = []

images = glob.glob('images/*.jpg')

print(f"Found {len(images)} images")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)

    if ret:
        corners = cv2.cornerSubPix(
            gray, corners, (3,3), (-1,-1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
        )

        objpoints.append(objp)
        imgpoints.append(corners)

        cv2.drawChessboardCorners(img, CHECKERBOARD, corners, ret)
        cv2.imshow("Corners", img)
        cv2.waitKey(100)
    else:
        print(f"Not detected: {fname}")

cv2.destroyAllWindows()

# ===== CALIBRATION =====
K = np.zeros((3, 3))
D = np.zeros((4, 1))

rvecs = []
tvecs = []

ret, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
    objpoints,
    imgpoints,
    gray.shape[::-1],
    K,
    D,
    rvecs,
    tvecs,
    cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC,
    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
)

print("\n=== RESULTS ===")
print("Camera matrix:\n", K)
print("\nDistortion coefficients:\n", D)

# ===== UNDISTORT TEST =====
img = cv2.imread(images[0])
h, w = img.shape[:2]

map1, map2 = cv2.fisheye.initUndistortRectifyMap(
    K, D, np.eye(3), K, (w, h), cv2.CV_16SC2
)

undistorted = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR)

cv2.imshow("Original", img)
cv2.imshow("Undistorted", undistorted)
cv2.waitKey(0)
cv2.destroyAllWindows()