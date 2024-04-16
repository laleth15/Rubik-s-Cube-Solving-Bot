import cv2 as cv
import numpy as np
from cv2 import aruco
import argparse
from collections import defaultdict
from helpers import load_dict, load_calib_data, create_detector_params, get_marker_positions_from_base_marker, get_marker_size

ID_TO_COLOR = {
    0: (0, 255, 0),
    1: (0, 165, 255),
    2: (0, 255, 255),
    3: (0, 0, 255),
    4: (255, 0, 0),
    5: (255, 255, 255),
    6: (255, 100, 255),
    7: (255, 100, 255),
    8: (255, 100, 255),
    9: (255, 100, 255)
}



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

    last_base_marker_tvec = None
    last_base_marker_rvec = None

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        marker_corners, marker_IDs, reject = aruco.detectMarkers(
            gray_frame, marker_dict, parameters=detector_params
        )

        if marker_corners:
            # from the docs:
            # "The camera pose relative to the marker is a 3d transformation from the marker coordinate system to the camera coordinate system."
            
            marker_rvecs = defaultdict()
            marker_tvecs = defaultdict()

            for i in range(len(marker_IDs)):
                marker_ID = marker_IDs[i][0]
                marker_size = get_marker_size(marker_ID)
                rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                    marker_corners[i], marker_size, cam_mat, dist_coef
                )
                marker_rvecs[marker_ID] = rVec[0][0]
                marker_tvecs[marker_ID] = tVec[0][0]

            for marker_ID, rVec in marker_rvecs.items():
                tVec = marker_tvecs[marker_ID]
                marker_size = get_marker_size(marker_ID)
                # if marker_ID < 6:
                #     # frame = draw_cube(frame, rVec, tVec, cam_mat, dist_coef, ID_TO_COLOR[marker_ID], sidelength=5.5)
                #     point = cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec, tVec, marker_size, 4)
                # else:
                #     # frame = draw_cube(frame, rVec, tVec, cam_mat, dist_coef, ID_TO_COLOR[marker_ID], sidelength=marker_size)
                #     point = cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec, tVec, marker_size, 4)
                if marker_ID == 9:
                    last_base_marker_tvec = tVec
                    last_base_marker_rvec = rVec
                    point = cv.drawFrameAxes(frame, cam_mat, dist_coef, last_base_marker_rvec, last_base_marker_tvec, marker_size, 4)

            if last_base_marker_tvec is not None and last_base_marker_rvec is not None and len(marker_IDs) >= 1:
                composedRvecs, composedTvecs = get_marker_positions_from_base_marker(last_base_marker_rvec, last_base_marker_tvec, marker_rvecs, marker_tvecs, debug=True)
                baseRvec = last_base_marker_rvec
                baseTvec = last_base_marker_tvec

                for marker_ID, composedRvec in composedRvecs.items():
                    composedTvec = composedTvecs[marker_ID]

                    info = cv.composeRT(composedRvec, composedTvec, baseRvec.T, baseTvec.T)
                    TcomposedRvec, TcomposedTvec = info[0], info[1]

                    # print(f"Marker {marker_ID} position: ")
                    # print(f"x: {TcomposedTvec[0]} --- {marker_tvecs[marker_ID][0]}")
                    # print(f"y: {TcomposedTvec[1]} --- {marker_tvecs[marker_ID][1]}")
                    # print(f"z: {TcomposedTvec[2]} --- {marker_tvecs[marker_ID][2]}")

                    cv.drawFrameAxes(frame, cam_mat, dist_coef, TcomposedRvec, TcomposedTvec, 1.5, 4)


        cv.imshow("frame", frame)
        key = cv.waitKey(1)
        if key == ord("q"):
            break
        
    cap.release()
    cv.destroyAllWindows()
