import cv2 as cv
import numpy as np
from cv2 import aruco
import argparse
from collections import defaultdict
from helpers import load_dict, load_calib_data, create_detector_params, get_marker_positions_from_base_marker, get_marker_size, ID_TO_COLOR_STRING

def z_axis_direction_dict_to_up_face(z_axis_direction_dict):
    dot_products = {}
    for marker_id, tup in z_axis_direction_dict.items():
        z_axis, count = tup
        avg = np.average(z_axis, axis=0)
        dot_products[marker_id] = np.dot(avg, np.array([0,0,1]))

    return max(dot_products, key=dot_products.get)

def z_axis_direction_dict_to_camera_side_face(z_axis_direction_dict):
    dot_products = {}
    for marker_id, tup in z_axis_direction_dict.items():
        z_axis, count = tup
        avg = np.average(z_axis, axis=0)
        dot_products[marker_id] = np.dot(avg, np.array([0,1,0]))

    return max(dot_products, key=dot_products.get)

def get_up_marker_id_camera_marker_id(num_steps):
    ID_TO_Z_AXIS = {
        0: (np.zeros((num_steps,3)), 0),
        1: (np.zeros((num_steps,3)), 0),
        2: (np.zeros((num_steps,3)), 0),
        3: (np.zeros((num_steps,3)), 0),
        4: (np.zeros((num_steps,3)), 0),
        5: (np.zeros((num_steps,3)), 0),
        # just incase something is briefly mis-classified
        6: (np.zeros((num_steps,3)), 0),
        7: (np.zeros((num_steps,3)), 0),
        8: (np.zeros((num_steps,3)), 0),
        9: (np.zeros((num_steps,3)), 0),
    }

    ids_seen = set()

    last_base_marker_tvec = None
    last_base_marker_rvec = None

    for i in range(num_steps):
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
                if marker_ID == 9:
                    last_base_marker_tvec = tVec
                    last_base_marker_rvec = rVec
                    point = cv.drawFrameAxes(frame, cam_mat, dist_coef, last_base_marker_rvec, last_base_marker_tvec, marker_size, 4)

            if last_base_marker_tvec is not None and last_base_marker_rvec is not None and len(marker_IDs) >= 1:
                composedRvecs, composedTvecs = get_marker_positions_from_base_marker(last_base_marker_rvec, last_base_marker_tvec, marker_rvecs, marker_tvecs, debug=False)
                baseRvec = last_base_marker_rvec
                baseTvec = last_base_marker_tvec

                for marker_ID, composedRvec in composedRvecs.items():
                    composedTvec = composedTvecs[marker_ID]

                    info = cv.composeRT(composedRvec, composedTvec, baseRvec.T, baseTvec.T)
                    TcomposedRvec, TcomposedTvec = info[0], info[1]

                    # orientation code
                    z_axis = np.array([0,0,1])
                    rotV, _ = cv.Rodrigues(composedRvec)
                    new_z = np.around(rotV @ z_axis, 0)

                    values, count = ID_TO_Z_AXIS[marker_ID]
                    count += 1

                    new_values = np.copy(values)
                    new_values[count % num_steps] = new_z

                    ID_TO_Z_AXIS[marker_ID] = (new_values, count)

                    ids_seen.add(marker_ID)

                    cv.drawFrameAxes(frame, cam_mat, dist_coef, TcomposedRvec, TcomposedTvec, 1.5, 4)

                # was using this to refresh every step for things not yet seen
                # for marker_ID in range(6):
                #     if marker_ID not in composedRvecs:
                #         values, count = ID_TO_Z_AXIS[marker_ID]
                #         values[count % num_steps] = np.zeros(3)
                #         count += 1
                #         ID_TO_Z_AXIS[marker_ID] = (values, count)

        cv.imshow("frame", frame)
        key = cv.waitKey(1)
        # # if key == ord("q"):
        # #     break

    up_id = z_axis_direction_dict_to_up_face(ID_TO_Z_AXIS)
    side_id = z_axis_direction_dict_to_camera_side_face(ID_TO_Z_AXIS)

    print(f"Up marker: {ID_TO_COLOR_STRING[up_id]}")
    print(f"Side marker: {ID_TO_COLOR_STRING[side_id]}")
    print(f"Num IDs seen: {len(ids_seen)}")
    return up_id, side_id    


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


    up_id, side_id = get_up_marker_id_camera_marker_id(50)


    cap.release()
    cv.destroyAllWindows()
