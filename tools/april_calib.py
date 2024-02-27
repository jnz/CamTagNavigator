# 2024, Jan Zwiener (jan@zwiener.org)
#
# Script to calibrate a camera based on images with a known
# marker database.
#
# Dependencies:
#   pip install opencv-python
#   pip install apriltag

import cv2
import numpy as np
import apriltag
import glob
import xml.etree.cElementTree as ET
from datetime import datetime

# Function to read the 3D positions of the AprilTags from a marker database
def read_tag_positions(filename):
    tag_positions = {}
    with open(filename, 'r') as file:
        for line in file:
            parts = line.strip().split()
            tag_id = int(parts[0])
            coords = np.array(parts[1:], dtype=float).reshape(4, 3)  # Reshape to 4x3 for each corner
            tag_positions[tag_id] = coords
    return tag_positions

output_file = 'actioncam.xml'

# Initialize the AprilTag detector
detector = apriltag.Detector()

# Path to images and the marker position database
image_files = glob.glob('../example/*.jpg')
tag_position_file = '../example/00_room.txt'

# Read the known 3D positions of the tags
tag_positions = read_tag_positions(tag_position_file)

# Lists to hold the corresponding points
object_points = []  # 3D points in real world space
image_points = []  # 2D points in image plane

# Process each image
for image_file in image_files:
    image = cv2.imread(image_file)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(gray)

    num_tags = len(tags)
    obj_pts = np.zeros((num_tags * 4, 3), np.float32)
    img_pts = np.zeros((num_tags * 4, 2), np.float32)
    idx = 0

    if num_tags < 8:
        continue

    for tag in tags:
        # Check if the detected tag ID is in our list of known positions
        if tag.tag_id in tag_positions:
            p3d = tag_positions[tag.tag_id]  # The 3D points of this tag
            p2d = np.array(tag.corners, dtype=np.float32)  # The detected 2D corners

            for i in range(0, 4):
                obj_pts[idx][:] = p3d[i][:]
                img_pts[idx][:] = p2d[i][:]
                idx += 1

            # Draw the detected tag's boundaries and ID on the image
            for i in range(4):
                pt1 = tuple(np.int32(tag.corners[i]))
                pt2 = tuple(np.int32(tag.corners[(i+1) % 4]))
                cv2.line(image, pt1, pt2, (0, 255, 0), 2)
            cv2.putText(image, str(tag.tag_id), tuple(np.int32(tag.center)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    object_points.append(obj_pts)
    image_points.append(img_pts)

    cv2.imshow('Detected Tags', image)
    cv2.waitKey(10)

cv2.destroyAllWindows()

image_size = (image.shape[1], image.shape[0])
# Initialize the intrinsic matrix with a reasonable guess For fx and fy, using
# the average of image width and height can be a starting point cx and cy are
# set to the center of the image
image_width, image_height = image_size
fx = fy = 0.5 * (image_width + image_height)  # Adjust based on your estimation
cx, cy = image_width / 2, image_height / 2
intrinsic_matrix_guess = np.array([[fx, 0, cx],
                                   [0, fy, cy],
                                   [0, 0, 1]], dtype="double")

# Initialize distortion coefficients - assuming minimal distortion initially
dist_coeffs_guess = np.zeros(5)  # Assume OpenCV expects 5 coefficients in the basic model

# Camera calibration with the initial guess for the intrinsic matrix and
# distortion coefficients
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points,
                                                   image_size,
                                                   intrinsic_matrix_guess,
                                                   dist_coeffs_guess,
                                                   flags=cv2.CALIB_USE_INTRINSIC_GUESS)

print(f"Camera matrix: \n{mtx}")
print(f"Distortion coefficients: {dist}")
print(f"Reprojection error: {ret}")

# for image_file in image_files:
#     image = cv2.imread(image_file)
#
#     undistorted_image = cv2.undistort(image, mtx, dist, None, mtx)
#
#     # Display the original and undistorted images
#     cv2.imshow("Original Image", image)
#     cv2.imshow("Undistorted Image", undistorted_image)
#
#     cv2.waitKey(1000)
#
# cv2.destroyAllWindows()

# Save calibration to a .xml file:
calibration_time = datetime.now().strftime("%a %b %d %H:%M:%S %Y")
# XML structure
opencv_storage = ET.Element("opencv_storage")
ET.SubElement(opencv_storage, "calibration_time").text = f'"{calibration_time}"'
ET.SubElement(opencv_storage, "image_width").text = str(image_width)
ET.SubElement(opencv_storage, "image_height").text = str(image_height)
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
tree.write(output_file, encoding="utf-8", xml_declaration=True)
print("Calibration file '%s' created." % (output_file))

