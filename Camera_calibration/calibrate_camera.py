import numpy as np
import cv2
import glob
import sys
import yaml

"""
After capturing images of a chessboard pattern, this script calibrates the camera
and saves the calibration parameters to a YAML file.

Usage: python3 calibrate_camera.py [image_dir] [output_yaml]
e.g.   
"""

# Chessboard settings
CHESSBOARD_SIZE = (9, 6)  # (rows, columns)
SQUARE_SIZE = 8.55  # mm

IMAGE_DIR = sys.argv[1] if len(sys.argv) > 1 else "calib_images_front"
OUT_FILE = sys.argv[2] if len(sys.argv) > 2 else "calibration.yaml"

# Prepare object points based on the chessboard size and square size
# objp will hold the 3D coordinates of the chessboard corners in the world space
objp = np.zeros((CHESSBOARD_SIZE[0]*CHESSBOARD_SIZE[1], 3), np.float32)

# Generate the grid points in the chessboard pattern
# objp[:, :2] will hold the x, y coordinates of the corners
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)

# Scale the points by the size of each square
objp *= SQUARE_SIZE

# Arrays to store points
objpoints = []  # 3D points
imgpoints = []  # 2D points

# Load images from the specified directory
images = sorted(glob.glob(f"{IMAGE_DIR}/*.jpg"))

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    found, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

    # If corners are found, continue
    if found:
        objpoints.append(objp)
        # Refine the corner locations
        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )
        # Append the refined corners to imgpoints list
        imgpoints.append(corners2)
    else:
        print(f"No chessboard found in {fname}, skipping")

if not objpoints:
    sys.exit(f"No usable images in {IMAGE_DIR}/ - collect images first "
             "with collecting_chessboard_images.py")

print(f"Calibrating from {len(objpoints)}/{len(images)} images...")

# Calibrate the camera using the collected object points and image points
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

# print the calibration results
print("Camera matrix:\n", mtx)
print("Distortion coefficients:\n", dist)
print("Reprojection error:", ret)

calib_data = {
    "camera_matrix": mtx.tolist(),
    "dist_coeff": dist.tolist(),
    "reprojection_error": float(ret)
}

# Save the calibration data to a YAML file
with open(OUT_FILE, "w") as f:
    yaml.dump(calib_data, f)

print(f"Saved {OUT_FILE}")

# import cv2
# import numpy as np
# import glob

# # Checkerboard dimensions (inner corners)
# CHECKERBOARD = (9,6)

# # Size of one square (millimeters)
# square_size = 20.0

# criteria = (
#     cv2.TERM_CRITERIA_EPS +
#     cv2.TERM_CRITERIA_MAX_ITER,
#     30,
#     0.001
# )

# objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1],3), np.float32)

# objp[:,:2] = np.mgrid[
#     0:CHECKERBOARD[0],
#     0:CHECKERBOARD[1]
# ].T.reshape(-1,2)

# objp *= square_size

# objpoints = []
# imgpoints = []

# images = glob.glob("calib_images/*.jpg")

# for fname in images:

#     img = cv2.imread(fname)

#     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

#     ret, corners = cv2.findChessboardCorners(
#         gray,
#         CHECKERBOARD,
#         None
#     )

#     if ret:

#         objpoints.append(objp)

#         corners2 = cv2.cornerSubPix(
#             gray,
#             corners,
#             (11,11),
#             (-1,-1),
#             criteria
#         )

#         imgpoints.append(corners2)

#         cv2.drawChessboardCorners(
#             img,
#             CHECKERBOARD,
#             corners2,
#             ret
#         )

#         cv2.imshow("Corners", img)
#         cv2.waitKey(300)

# cv2.destroyAllWindows()

# ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(
#     objpoints,
#     imgpoints,
#     gray.shape[::-1],
#     None,
#     None
# )

# print("Camera Matrix:")
# print(cameraMatrix)

# print("\nDistortion:")
# print(distCoeffs)

# np.save("cameraMatrix.npy", cameraMatrix)
# np.save("distCoeffs.npy", distCoeffs)