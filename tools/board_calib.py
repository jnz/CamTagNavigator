# 2024, Jan Zwiener (jan@zwiener.org)
#
# Script to calibrate a camera based on a checkerboard pattern.
#
# Dependencies:
#   pip install opencv-python

import cv2
import numpy as np
import glob
import xml.etree.cElementTree as ET
from datetime import datetime

# Chessboard settings
board_width = 9
board_height = 7
square_size = 0.021 # meters
pattern_size = (board_width, board_height)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points
objp = np.zeros((board_height*board_width, 3), np.float32)
objp[:,:2] = np.mgrid[0:board_width, 0:board_height].T.reshape(-1, 2) * square_size

# Arrays to store object points and image points
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane

# Start camera
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

print("Press 's' to use image for calibration")
print("Press 'q' to stop capturing and proceed with calibration")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

    # If found, add object points, image points
    if ret == True:
        corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        cv2.drawChessboardCorners(frame, pattern_size, corners2, ret)

    cv2.imshow('frame', frame)
    key = cv2.waitKey(1)

    if key == ord('s'): # Save on 's' key
        imgpoints.append(corners2)
        objpoints.append(objp)
        print(f"Image saved: {len(imgpoints)}")

    if key == ord('q'): # Quit and calibrate on 'q' key
        break

# Release everything if job is finished
cap.release()
cv2.destroyAllWindows()

# Camera calibration
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Saving the calibration results
calibration_time = datetime.now().strftime("%a %b %d %H:%M:%S %Y")
image_width = frame.shape[1]
image_height = frame.shape[0]

print("Finished. Reprojection error: %.2f" % (ret))
if ret < 0.5:
    print("Excellent calibration quality.")
elif ret < 1.0:
    print("Good calibration quality. Acceptable for high-precision applications.")
elif ret < 2.0:
    print("Moderate calibration quality. Might be sufficient for less demanding applications.")
else:
    print("Poor calibration result. Consider recalibration.")

# Write out calibration parameters to XML file
opencv_storage = ET.Element("opencv_storage")
ET.SubElement(opencv_storage, "calibration_time").text = f'"{calibration_time}"'
ET.SubElement(opencv_storage, "image_width").text = str(image_width)
ET.SubElement(opencv_storage, "image_height").text = str(image_height)
ET.SubElement(opencv_storage, "board_width").text = str(board_width)
ET.SubElement(opencv_storage, "board_height").text = str(board_height)
ET.SubElement(opencv_storage, "square_size").text = str(square_size)
camera_matrix = ET.SubElement(opencv_storage, "camera_matrix", type_id="opencv-matrix")
ET.SubElement(camera_matrix, "rows").text = "3"
ET.SubElement(camera_matrix, "cols").text = "3"
ET.SubElement(camera_matrix, "dt").text = "d"
ET.SubElement(camera_matrix, "data").text = ' '.join(map(str, mtx.flatten()))
distortion_coefficients = ET.SubElement(opencv_storage, "distortion_coefficients", type_id="opencv-matrix")
ET.SubElement(distortion_coefficients, "rows").text = str(len(dist.flatten()))
ET.SubElement(distortion_coefficients, "cols").text = "1"
ET.SubElement(distortion_coefficients, "dt").text = "d"
ET.SubElement(distortion_coefficients, "data").text = ' '.join(map(str, dist.flatten()))
ET.SubElement(opencv_storage, "avg_reprojection_error").text = str(ret)

tree = ET.ElementTree(opencv_storage)
tree.write("calibration.xml", encoding="utf-8", xml_declaration=True)
print("Calibration file 'calibration.xml' created.")

