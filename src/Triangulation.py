import sys
import cv2
import numpy as np
import time


def find_depth(rect_right, rect_left, img_right, img_left, baseline, alpha):
    height_right, width_right, depth_right = img_right.shape
    height_left, width_left, depth_left = img_left.shape

    # if width_right == width_left:
    f_pixel = (width_right * 0.5) / np.tan(alpha * 0.5 * np.pi/180)

    # else:
    #     print('Left and right camera frames do not have the same pixel width')

    x_right = rect_right[0]
    x_left = rect_left[0]

    # CALCULATE THE DISPARITY:
    disparity = x_left-x_right      # Displacement between left and right frames [pixels]

    # CALCULATE DEPTH z:
    zDepth = (baseline * f_pixel)/disparity             #Depth in [cm]

    return abs(zDepth)