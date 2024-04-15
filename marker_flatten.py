import cv2 as cv
import numpy as np
from cv2 import aruco
import argparse
from collections import defaultdict
import twophase.solver as sv
from helpers import load_dict, load_calib_data, create_detector_params, get_marker_size

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


colors = {
    'w': (200, 200, 210), # white
    'y': (80, 200, 205), # yellow
    'g': (75, 140, 50), # green
    'b': (120, 75, 20), # blue
    'o': (50, 90, 230), # orange
    'r': (50, 40, 190), # red
}

marker_to_color = {
    0: 'w',
    1: 'r',
    2: 'b',
    3: 'g',
    4: 'o',
    5: 'y'
}

color_to_marker = {
    'w': 0,
    'r': 1,
    'b': 2,
    'g': 3,
    'o': 4,
    'y': 5
}


color_to_side = {
    'w': 'U',
    'y': 'D',
    'g': 'F',
    'b': 'B',
    'r': 'R',
    'o': 'L'
}

def normalize(v):
    return v / np.linalg.norm(v, ord=2)

def get_color(r,g,b): # compare rgb values and return color
    min_distance = np.inf
    best_color = None

    input = normalize(np.array((r, g, b)))

    for (key, color) in colors.items():
        dist = np.linalg.norm(input - normalize(np.array(color)), ord=1)
        if dist < min_distance:
            min_distance = dist
            best_color = key, color
    return best_color

def convert_to_string(cube_array):
    output = ''

    order = [0, 1, 3, 5, 4, 2]

    for k in range(6):
        for i in range(3):
            for j in range(3):
                output += color_to_side[marker_to_color[cube_array[order[k]][j][i]]]

    return output

class CubeScanner:
    def __init__(self, camera_id, camera_name):
        self.marker_dict = load_dict()
        self.cam_mat, self.dist_coef = load_calib_data(camera_name)
        self.detector_params = create_detector_params()

        # thank you https://answers.opencv.org/question/41899/changing-pixel-format-yuyv-to-mjpg-when-capturing-from-webcam/
        self.cap = cv.VideoCapture(CAMERA_NUMBER)
        # self.cap = cv.VideoCapture(CAMERA_NUMBER, cv.CAP_V4L2)
        # self.cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        # self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 1920)
        # self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)
        self.camera_id = camera_id
        self.camera_name = camera_name
        
        self.cube = np.full((6, 3, 3), -1)
        self.prev_count = 30
        self.prevs = np.full((self.prev_count, 6, 3, 3), -1)

    def end_scanning(self):
        self.cap.release()
        cv.destroyAllWindows()

    def get_solution(self):
        cube_string = convert_to_string(self.cube)
        return sv.solve(cube_string)

    # rotation_index is number of times to rotate face by 90 degress counter-clockwise
    def scan_face(self, face_color, rotation_index):
        marker_id = color_to_marker[face_color]
        count = 0

        def is_full():
            for instance in self.prevs:
                for row in instance[marker_id]:
                    for sticker in row:
                        if sticker == -1:
                            return False
            return True
        
        def prev_equal():
            for i in range(self.prevs.shape[0] - 1):
                for j in range(3):
                    for k in range(3):
                        if self.prevs[i][marker_id][j][k] != self.prevs[i + 1][marker_id][j][k]:
                            return False
            return True

        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            marker_corners, marker_IDs, _ = aruco.detectMarkers(
                gray_frame, self.marker_dict, parameters=self.detector_params
            )
            current_marker = marker_IDs[0][0] if marker_IDs is not None and len(marker_IDs) > 0 else None

            if current_marker != marker_id: continue

            self.prevs[:, :, 1, 1] = marker_id

            if current_marker is not None:
                # Needed to ignore marker orientation
                smallest_point_index = np.argmin(np.sum(marker_corners[0][0], axis=1))
                marker_corners[0][0] = np.roll(marker_corners[0][0], rotation_index + 3 - smallest_point_index, axis=0)

                transform = cv.getPerspectiveTransform(np.float32(marker_corners[0]), np.array([[200, 200], [200, 300], [300, 300], [300, 200]], dtype=np.float32))
                frame = cv.warpPerspective(frame, transform,(500, 500),flags=cv.INTER_LINEAR)

            center = (250, 250)
            square_size = 60
            half_square = int(square_size / 2)
            gap = 50

            frame = cv.flip(frame, 1)

            frame2 = frame.copy()
            display = np.zeros((500, 500, 3))

            for j in range(3):
                startY = center[1] - half_square - gap - square_size + j * (gap + square_size)
                endY = startY + square_size

                for i in range(3):
                    startX = center[0] - half_square - gap - square_size + i * (gap + square_size)
                    endX = startX + square_size

                    try:
                        cv.rectangle(frame, (startX, startY), (endX, endY), (255,255,255), 3)

                        if i == j == 1:
                            display[startY:endY, startX:endX] = colors[marker_to_color[current_marker]]
                            self.cube[current_marker][i][j] = current_marker
                            continue

                        if current_marker is None: continue

                        subframe = frame2[startY:endY, startX:endX]

                        r, g, b = cv.split(subframe)
                        r_avg = int(cv.mean(r)[0])
                        g_avg = int(cv.mean(g)[0])
                        b_avg = int(cv.mean(b)[0])
                        # print(r_avg, g_avg, b_avg)
                        key, color = get_color(r_avg, g_avg, b_avg)

                        marker = color_to_marker[key]
                        self.prevs[count][current_marker][i][j] = marker

                        if np.all(self.prevs[:, current_marker, i, j] == marker):
                            display[startY:endY, startX:endX] = color
                        
                    except:
                        continue

            cv.imshow("frame", frame)
            cv.imshow("display", display / 255)

            key = cv.waitKey(1)
            if key == ord("q"):
                break
            
            if is_full() and prev_equal():
                self.cube[current_marker] = self.prevs[0][current_marker]
                print(self.prevs[:, marker_id])
                print(f'{face_color} side done')
                return

            count += 1
            count %= self.prevs.shape[0]


def re_orient_cube(top_id, front_id):
    top_color = marker_to_color[top_id]
    front_color = marker_to_color[front_id]
    value = top_color + front_color
    mapper = {
        'wg': "y'",
        'wr': "y2",
        'wb': "y",
        'wo': "",

        'gw': 'z',
        'go': "x'",
        'gy': "z'",
        'gr': "x",

        'rg': "x' y'",
        'ry': "z' y'",
        'rb': "x y'",
        'rw': "z y'",

        'bo': "x y2",
        'bw': 'z y2',
        'br': "x' y2",
        'by': "z' y2",

        'yg': "z2 y'",
        'yo': "z2",
        'yb': 'z2 y',
        'yr': 'z2 y2',

        'ob': "x' y",
        'oy': "z' y",
        'og': "x y",
        'ow': "z y",
    }

    return mapper[value].split(' ')
        
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

    cube_scanner = CubeScanner(CAMERA_NUMBER, CAMERA_NAME)

    cube_scanner.scan_face('b', 3)
    cube_scanner.scan_face('g', 3) # scan yellow green bottom
    cube_scanner.scan_face('o', 3)
    cube_scanner.scan_face('r', 3)
    cube_scanner.scan_face('y', 2)
    cube_scanner.scan_face('w', 0)
    print(cube_scanner.cube)
    cube_scanner.end_scanning()

    print(cube_scanner.get_solution())


    
