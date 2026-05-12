import cv2
import numpy as np
import glob

CHECKERBOARD = (8, 5)
square_size = 0.025  # meters (adjust if needed)

# object points (3D real world)
objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = []
imgpoints = []

images = glob.glob("calib_images/*.png")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD)

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

print("Images found:", len(images))

# Calibration
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD)

    print(fname, "detected:", ret)  # <-- ADD THIS

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)