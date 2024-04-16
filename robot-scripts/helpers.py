import numpy as np
import cv2 as cv
import numpy as np
from cv2 import aruco
from collections import defaultdict

def load_dict():
    data = np.load('../markers-dict/dictionary.npz')
    loaded_dict = cv.aruco.Dictionary_create(10,3)
    loaded_dict.bytesList = data['bytesList']
    return loaded_dict

def load_calib_data(camera_name):
    calib_data_path = f"../calib-data/MultiMatrix_Camera_{camera_name}.npz"
    calib_data = np.load(calib_data_path)
    cam_mat = calib_data["camMatrix"]
    dist_coef = calib_data["distCoef"]
    return cam_mat, dist_coef

def create_detector_params():
    adaptiveThreshConstant = 7
    adaptiveThreshWinSizeMin = 3
    adaptiveThreshWinSizeMax = 18
    adaptiveThreshWinSizeStep = 3
    detector_params = aruco.DetectorParameters_create()
    # detector_params.adaptiveThreshConstant = adaptiveThreshConstant
    detector_params.adaptiveThreshWinSizeMin = adaptiveThreshWinSizeMin
    detector_params.adaptiveThreshWinSizeMax = adaptiveThreshWinSizeMax
    detector_params.adaptiveThreshWinSizeStep = adaptiveThreshWinSizeStep
    return detector_params

def draw_cube(img, R, t, camera_mat, dist_coef, color, sidelength = 5.6):
    '''
    Referenced from: 
        How to draw 3D Coordinate Axes with OpenCV for face pose estimation?
        https://stackoverflow.com/questions/30207467/how-to-draw-3d-coordinate-axes-with-opencv-for-face-pose-estimation
    '''
    # unit is mm
    rotV, _ = cv.Rodrigues(R)
    points = np.float32([[-sidelength/2, sidelength/2, 0],  [-sidelength/2,-sidelength/2,0],[sidelength/2, -sidelength/2, 0], [sidelength/2, sidelength/2, 0], [-sidelength/2, sidelength/2, -sidelength],  [-sidelength/2,-sidelength/2, -sidelength],[sidelength/2, -sidelength/2, -sidelength], [sidelength/2, sidelength/2, -sidelength]]).reshape(-1, 3)
    axisPoints, _ = cv.projectPoints(points, rotV, t, camera_mat, dist_coef)

    # draw cube edges onto screen
    img = cv.line(img, tuple(axisPoints[0].ravel().astype(int)), tuple(axisPoints[1].ravel().astype(int)), color, 3)    
    img = cv.line(img, tuple(axisPoints[1].ravel().astype(int)), tuple(axisPoints[2].ravel().astype(int)), color, 3)
    img = cv.line(img, tuple(axisPoints[2].ravel().astype(int)), tuple(axisPoints[3].ravel().astype(int)), color, 3)    
    img = cv.line(img, tuple(axisPoints[3].ravel().astype(int)), tuple(axisPoints[0].ravel().astype(int)), color, 3)

    img = cv.line(img, tuple(axisPoints[4].ravel().astype(int)), tuple(axisPoints[5].ravel().astype(int)), color, 3)    
    img = cv.line(img, tuple(axisPoints[5].ravel().astype(int)), tuple(axisPoints[6].ravel().astype(int)), color, 3)
    img = cv.line(img, tuple(axisPoints[6].ravel().astype(int)), tuple(axisPoints[7].ravel().astype(int)), color, 3)    
    img = cv.line(img, tuple(axisPoints[7].ravel().astype(int)), tuple(axisPoints[4].ravel().astype(int)), color, 3)
 
    img = cv.line(img, tuple(axisPoints[0].ravel().astype(int)), tuple(axisPoints[4].ravel().astype(int)), color, 3)    
    img = cv.line(img, tuple(axisPoints[1].ravel().astype(int)), tuple(axisPoints[5].ravel().astype(int)), color, 3)
    img = cv.line(img, tuple(axisPoints[2].ravel().astype(int)), tuple(axisPoints[6].ravel().astype(int)), color, 3)    
    img = cv.line(img, tuple(axisPoints[3].ravel().astype(int)), tuple(axisPoints[7].ravel().astype(int)), color, 3)
    
    return img

def inversePerspective(rvec, tvec):
    ''' 
    Get the inverse perspective of the rvec and tvec
    Referenced from:
    https://aliyasineser.medium.com/calculation-relative-positions-of-aruco-markers-eee9cc4036e3
    '''
    R, _ = cv.Rodrigues(rvec)
    R = np.matrix(R).T
    invTvec = np.dot(R, np.matrix(-tvec))
    invRvec, _ = cv.Rodrigues(R)
    return invRvec, invTvec

def relativePosition(rvec1, tvec1, rvec2, tvec2):
    """ 
    Get relative position for rvec2 & tvec2. Compose the returned rvec & tvec to use composeRT with rvec2 & tvec2 
    Referenced from:
    https://aliyasineser.medium.com/calculation-relative-positions-of-aruco-markers-eee9cc4036e3
    """
    rvec1, tvec1 = rvec1.reshape((3, 1)), tvec1.reshape((3, 1))
    rvec2, tvec2 = rvec2.reshape((3, 1)), tvec2.reshape((3, 1))
    # Inverse the second marker
    invRvec, invTvec = inversePerspective(rvec2, tvec2)
    info = cv.composeRT(rvec1, tvec1, invRvec, invTvec)
    composedRvec, composedTvec = info[0], info[1]
    composedRvec = composedRvec.reshape((3, 1))
    composedTvec = composedTvec.reshape((3, 1))
    return composedRvec, composedTvec

def get_marker_positions_from_base_marker(base_marker_rvec, base_marker_tvec, marker_rvecs, marker_tvecs, debug=True):
    '''
    Get the position of the markers from the base marker
    '''
    base_marker_id = 9

    output_rvecs = defaultdict()
    output_tvecs = defaultdict()

    for marker_id, rvec in marker_rvecs.items():
        if marker_id != base_marker_id:
            tvec = marker_tvecs[marker_id]
            composedRvec, composedTvec = relativePosition(rvec, tvec,base_marker_rvec, base_marker_tvec)
            output_rvecs[marker_id] = composedRvec.ravel()
            output_tvecs[marker_id] = composedTvec.ravel()
            if debug:
                # print(f"Marker {marker_id} position: ")
                # print(f"x: {composedTvec[0][0]}")
                # print(f"y: {composedTvec[1][0]}")
                # print(f"z: {composedTvec[2][0]}")
                print(f"Marker {marker_id} rotation: ")
                print(f"x: {composedRvec[0][0]}")
                print(f"y: {composedRvec[1][0]}")
                print(f"z: {composedRvec[2][0]}")

    return output_rvecs, output_tvecs


LARGE_MARKER_SIZE = 3 # centimeters

def get_marker_size(marker_id):
    '''
    Get the marker size in centimeters
    '''
    if marker_id < 6:
        return 1.5
    else:
        return LARGE_MARKER_SIZE
    

ID_TO_COLOR_STRING = {
    0: "white",
    1: "red",
    2: "blue",
    3: "green",
    4: "orange",
    5: "yellow",
    6: "base",
    7: "base",
    8: "base",
    9: "base",
}
