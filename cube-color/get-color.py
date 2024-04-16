import cv2 as cv
import numpy as np
from cv2 import aruco
import argparse
from collections import defaultdict


def load_calib_data(camera_number):
    calib_data_path = (
        f"../april-tags-testing/calib-data/MultiMatrix_Camera_{camera_number}.npz"
    )
    calib_data = np.load(calib_data_path)
    cam_mat = calib_data["camMatrix"]
    dist_coef = calib_data["distCoef"]
    return cam_mat, dist_coef


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Get the position of the marker")
    parser.add_argument("-c", "--camera", type=int, help="Enter camera number")
    args = parser.parse_args()

    if args.camera is None:
        print("Please enter camera number using -c or --camera")
        exit()
    else:
        CAMERA_NUMBER = args.camera
        print(f"Using camera number {CAMERA_NUMBER}")

    cam_mat, dist_coef = load_calib_data(CAMERA_NUMBER)
    cap = cv.VideoCapture(CAMERA_NUMBER)

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        marker_corners, marker_IDs, reject = aruco.detectMarkers(
            gray_frame, marker_dict, parameters=detector_params
        )

        if marker_corners:

            for corners, marker_id in zip(marker_corners, marker_IDs):
                transform = cv.getPerspectiveTransform(np.float32(corners), np.array([[200, 200], [200, 300], [300, 300], [300, 200]], dtype=np.float32))
                frame = cv.warpPerspective(frame, transform,(500, 500),flags=cv.INTER_LINEAR)
                break

        # cv.imshow("frame", frame)
        key = cv.waitKey(1)
        if key == ord("q"):
            break

    cap.release()
    cv.destroyAllWindows()
