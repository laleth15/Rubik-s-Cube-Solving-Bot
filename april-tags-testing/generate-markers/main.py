import cv2 as cv
from cv2 import aruco
import numpy as np
from inspect import getmembers

MARKER_SIZE = 600  # pixels

marker_dict = aruco.Dictionary_create(10, 3)
# np.savez_compressed('../markers-dict/dictionary.npz', bytesList=marker_dict.bytesList, markerSize=marker_dict.markerSize)

data = np.load('../markers-dict/dictionary.npz')
loaded_dict = cv.aruco.Dictionary_create(10,3)
loaded_dict.bytesList = data['bytesList']

for id in range(10):
    marker_image = aruco.drawMarker(marker_dict, id, MARKER_SIZE)
    cv.imshow("img", marker_image)
    # cv.imwrite(f"markers/marker_{id}.png", marker_image)
    cv.waitKey(0)