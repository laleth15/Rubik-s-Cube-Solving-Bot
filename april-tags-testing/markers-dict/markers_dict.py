import cv2 as cv
from cv2 import aruco
import numpy as np

def dict_loader(): 
    data = np.load('./dictionary.npz')
    loaded_dict = cv.aruco.Dictionary_create(10,3)
    loaded_dict.bytesList = data['bytesList']
    return loaded_dict