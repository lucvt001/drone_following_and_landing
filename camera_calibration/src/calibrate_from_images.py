import cv2
import numpy as np
import glob
import yaml

def calibrate_camera(image_dir, chessboard_size, square_size, output_yaml):
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare object points
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    objp *= square_size

    # Arrays to store object points and image points from all the images
    objpoints = []  # 3d points in real world space
    imgpoints = []  # 2d points in image plane

    # Get list of images
    images = glob.glob(f'{image_dir}/*.jpg')

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            # cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
            # cv2.imshow('Chessboard', img)
            # cv2.waitKey(500)

    cv2.destroyAllWindows()

    # Perform camera calibration
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # Compute reprojection error
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error

    reprojection_error = mean_error / len(objpoints)
    print("Reprojection Error: ", reprojection_error)

    # Save the coefficients to a YAML file
    calibration_data = {
        'board_Width': chessboard_size[0],
        'board_Height': chessboard_size[1],
        'square_Size': square_size,
        'camera_matrix': camera_matrix.tolist(),
        'dist_coeffs': dist_coeffs.tolist(),
        'reprojection_error': reprojection_error,
        'rvecs': [rvec.tolist() for rvec in rvecs],
        'tvecs': [tvec.tolist() for tvec in tvecs],
    }

    with open(output_yaml, 'w') as f:
        yaml.dump(calibration_data, f)

    print(f"Calibration coefficients saved to {output_yaml}")

if __name__ == "__main__":
    image_dir = "/home/lucvt001/camera_calibration_cpp/image"  # Replace with your directory path
    chessboard_size = (15, 10)  # Number of inner corners per a chessboard row and column
    square_size = 0.020  # Size of a square in your defined unit (e.g., meters)
    output_yaml = "/home/lucvt001/drone_ws/src/camera_calibration/dwe_cam.yaml"

    calibrate_camera(image_dir, chessboard_size, square_size, output_yaml)