import argparse
import cv2 as cv
import os


def detect_checker_board(image, grayImage, criteria, boardDimension):
    ret, corners = cv.findChessboardCorners(grayImage, boardDimension)
    if ret == True:
        corners1 = cv.cornerSubPix(grayImage, corners, (3, 3), (-1, -1), criteria)
        image = cv.drawChessboardCorners(image, boardDimension, corners1, ret)

    return image, ret

def check_directory(dir_path):
    CHECK_DIR = os.path.isdir(dir_path)
    # if directory does not exist create
    if not CHECK_DIR:
        os.makedirs(dir_path)
        print(f'"{dir_path}" Directory is created')
    else:
        print(f'"{dir_path}" Directory already Exists.')


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Enter camera port number and camera name')
    parser.add_argument('-c','--camera', type=int, help='Enter camera number')
    parser.add_argument('-n','--name', type=str, help='Enter camera name')

    args = parser.parse_args()

    n = 0  # image_counter
    # check that camera number is provided or not
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

    CHECKERBOARD_DIM = (7, 7)

    # checking if  images dir is exist not, if not then create images directory
    image_dir_path = f"images_camera_{CAMERA_NAME}"
    check_directory(image_dir_path)

    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    # start video capture from camera
    # thank you https://answers.opencv.org/question/41899/changing-pixel-format-yuyv-to-mjpg-when-capturing-from-webcam/
    cap = cv.VideoCapture(CAMERA_NUMBER, cv.CAP_V4L2)
    cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)



    while True:
        _, frame = cap.read()
        copyFrame = frame.copy()
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        # image, board_detected = detect_checker_board(frame, gray, criteria, CHECKERBOARD_DIM)

        cv.putText(
            frame,
            f"saved_img : {n}",
            (30, 40),
            cv.FONT_HERSHEY_PLAIN,
            1.4,
            (0, 255, 0),
            2,
            cv.LINE_AA,
        )
        cv.putText(
            frame,
            "Press 's' to save the image",
            (30, 80),
            cv.FONT_HERSHEY_PLAIN,
            1.4,
            (0, 255, 0),
            2,
            cv.LINE_AA,
        )
        cv.putText(
            frame,
            "Press 'q' to exit",
            (30, 120),
            cv.FONT_HERSHEY_PLAIN,
            1.4,
            (0, 255, 0),
            2,
            cv.LINE_AA,
        )
        cv.putText(
            frame,
            f'Using camera number {CAMERA_NUMBER}',
            (30, 160),
            cv.FONT_HERSHEY_PLAIN,
            1.4,
            (0, 255, 0),
            2,
            cv.LINE_AA,
        )

        cv.imshow("frame", frame)
        key = cv.waitKey(1)
        if key == ord("q"):
            break


        board_detected = True
        if key == ord("s") and board_detected == True:
            # storing the checker board image
            cv.imwrite(f"{image_dir_path}/image{n}.png", copyFrame)

            print(f"saved image number {n}")
            n += 1  # incrementing the image counter

    cap.release()
    cv.destroyAllWindows()

    print("Total saved Images:", n)
