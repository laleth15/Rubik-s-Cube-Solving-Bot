import cv2 as cv
import numpy as np
from cv2 import aruco
import argparse
from collections import defaultdict
from helpers import load_dict, load_calib_data, create_detector_params, get_marker_positions_from_base_marker, get_marker_size, ID_TO_COLOR_STRING

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Get the position of the marker")
    parser.add_argument("-c", "--camera", type=int, help="Enter camera number")
    parser.add_argument("-n", "--name", type=str, help="Enter camera name")

    args = parser.parse_args()

    if args.camera is None:
        print("Please enter camera number using -c or --camera")
        exit()
    else:
        CAMERA_NUMBER = args.camera
        print(f"Using camera number {CAMERA_NUMBER}")

    if args.name is None:
        print("Please enter camera name using -n or --name")
        exit()
    else:
        CAMERA_NAME = args.name
        print(f"Using camera name {CAMERA_NAME}")

    marker_dict = load_dict()
    cam_mat, dist_coef = load_calib_data(CAMERA_NAME)

    detector_params = create_detector_params()

    # thank you https://answers.opencv.org/question/41899/changing-pixel-format-yuyv-to-mjpg-when-capturing-from-webcam/
    cap = cv.VideoCapture(CAMERA_NUMBER, cv.CAP_V4L2)
    cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)


    while True:
        ret, frame = cap.read()
        if not ret:
            break

        print(frame.shape)


        cv.imshow("frame", frame)
        key = cv.waitKey(1)
        if key == ord("q"):
            break
        


    cap.release()
    cv.destroyAllWindows()
