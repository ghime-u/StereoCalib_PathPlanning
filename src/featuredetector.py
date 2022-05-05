import cv2
import numpy as np
import glob
import sys
import os
# chessboard dimension

chessboardSize = (9, 6)
frameSize = (1440, 1080)

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0, 0, 0), (1, 0, 0), (2, 0, 0)
objpoints = np.zeros((chessboardSize[0]*chessboardSize[1], 3), np.float32)
objpoints[:, :2] = np.mgrid[0:chessboardSize[0], 0:chessboardSize[1]].T.reshape(-1, 2)
print(objpoints)

# arrays to store object(world) and image points for all images
objpoints_list = []
imgpoints_list = []

cap = cv2.VideoCapture(0)


def saveimageforcalib():
    count = 0
    while True:
        ret, image = cap.read()
        address = sys.argv[1]
        os.chdir(address)
        filename = "ss" + str(count)+".jpg"
        cv2.imshow('output', image)
        if cv2.waitKey(1) & 0xFF == ord('s'):
            cv2.imwrite(filename, image)
            count += 1
        elif cv2.waitKey(1) & 0xFF == ord('q'):
            break
        elif cv2.waitKey(1) & count == 2:
            print("Recieved all calibration images")
            break


def corners_for_calibration():
    # find chess board corners
    images = glob.glob('*.jpg')
    for i in images:
        img = cv2.imread(i)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, chessboardSize, None)
    if ret is True:
        objpoints_list.append(objpoints)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints_list.append(corners)
        # draw and display the corners
        cv2.drawChessboardCorners(img, chessboardSize, corners2, ret)
        cv2.imshow('output', img)
        cv2.waitKey(1)


def main():
    saveimageforcalib()
    corners_for_calibration()


if __name__ == '__main__':
    main()
