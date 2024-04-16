import cv2 as cv
import os
import argparse
import numpy as np


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Enter camera name')
    parser.add_argument('-n','--name', type=str, help='Enter camera name')

    args = parser.parse_args()

    # check that camera name is provided or not
    if args.name is None:
        print("Please enter camera name using -n or --name")
        exit()
    else:
        CAMERA_NAME = args.name
        print(f"Using camera name {CAMERA_NAME}")

    CHECKERBOARD_DIM = (7, 7)
    SQUARE_SIZE = 25.4

    
    # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # create directory to store calibration data
    calib_data_path = "../calib-data"
    CHECK_DIR = os.path.isdir(calib_data_path)
    if not CHECK_DIR:
        os.makedirs(calib_data_path)
        print(f'"{calib_data_path}" Directory is created')
    else:
        print(f'"{calib_data_path}" Directory already Exists.')

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    obj_3D = np.zeros((CHECKERBOARD_DIM[0] * CHECKERBOARD_DIM[1], 3), np.float32)

    obj_3D[:, :2] = np.mgrid[0 : CHECKERBOARD_DIM[0], 0 : CHECKERBOARD_DIM[1]].T.reshape(
        -1, 2
    )
    obj_3D *= SQUARE_SIZE
    print(obj_3D)

    # Arrays to store object points and image points from all the images.
    obj_points_3D = []  # 3d point in real world space
    img_points_2D = []  # 2d points in image plane.

    # The images directory path
    image_dir_path = image_dir_path = f"images_camera_{CAMERA_NAME}"

    files = os.listdir(image_dir_path)
    for file in files:
        print(file)
        imagePath = os.path.join(image_dir_path, file)

        image = cv.imread(imagePath)
        grayScale = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        ret, corners = cv.findChessboardCorners(image, CHECKERBOARD_DIM, None)
        if ret == True:
            obj_points_3D.append(obj_3D)
            corners2 = cv.cornerSubPix(grayScale, corners, (3, 3), (-1, -1), criteria)
            img_points_2D.append(corners2)

            img = cv.drawChessboardCorners(image, CHECKERBOARD_DIM, corners2, ret)

    cv.destroyAllWindows()
    # h, w = image.shape[:2]
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
        obj_points_3D, img_points_2D, grayScale.shape[::-1], None, None
    )
    print("calibrated")

    print("duming the data into one file using numpy ")
    np.savez(
        f"{calib_data_path}/MultiMatrix_Camera_{CAMERA_NAME}",
        camMatrix=mtx,
        distCoef=dist,
        rVector=rvecs,
        tVector=tvecs,
    )

    print("-------------------------------------------")

    print("loading data stored using numpy savez function\n \n \n")

    data = np.load(f"{calib_data_path}/MultiMatrix_Camera_{CAMERA_NAME}.npz")

    camMatrix = data["camMatrix"]
    distCof = data["distCoef"]
    rVector = data["rVector"]
    tVector = data["tVector"]

    print("loaded calibration data successfully")