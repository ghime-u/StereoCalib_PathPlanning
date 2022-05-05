import cv2
import numpy as np
import glob

# parameters

chessboardSize = (9, 6)
frameSize = (640, 480)

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# object points
objP = np.zeros((chessboardSize[0]*chessboardSize[1], 3), np.float32)
objP[:, :2] = np.mgrid[0:chessboardSize[0], 0:chessboardSize[1]].T.reshape((-1, 2))

# arrays to store object and image points

objPoints = []      # 3-D point in real world space
imgPointsL = []     # 2-D point in left image plane
imgPointsR = []     # 2-D point in right image plane

imagesLeft = glob.glob(r"C:\Users\HP\PycharmProjects\RRTPathPlanning\venv\L_Camera\*.png")
imagesRight = glob.glob(r"C:\Users\HP\PycharmProjects\RRTPathPlanning\venv\R_Camera\*.png")

for l_img, r_img in zip(imagesLeft, imagesRight):

    frameL = cv2.imread(l_img)
    frameR = cv2.imread(r_img)
    grayL = cv2.cvtColor(frameL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(frameR, cv2.COLOR_BGR2GRAY)

    # find chessboard corner
    retL, cornersL = cv2.findChessboardCorners(grayL, chessboardSize, None)
    retR, cornersR = cv2.findChessboardCorners(grayR, chessboardSize, None)

    # if found, add object and image points
    if retL and retR == True:

        objPoints.append(objP)

        cornersL = cv2.cornerSubPix(grayL, cornersL, (11,11), (-1,-1), criteria)
        imgPointsL.append(cornersL)

        cornersR = cv2.cornerSubPix(grayR, cornersR, (11, 11), (-1, -1), criteria)
        imgPointsR.append(cornersR)

        # draw and display corneres
        cv2.drawChessboardCorners(frameL, chessboardSize, cornersL, retL)
        cv2.imshow("left img", frameL)

        cv2.drawChessboardCorners(frameR, chessboardSize, cornersR, retR)
        cv2.imshow("right img", frameR)

cv2.destroyAllWindows()

# calibration

retL, cameraMatrixL, distL, rvecsL, tvecsL = cv2.calibrateCamera(objPoints, imgPointsL, frameSize, None, None)
np.savetxt('cameraMatrixL.csv', cameraMatrixL, delimiter=",")
heightL, widthL, channelsL = frameL.shape
newCameraMatrixL, roi_L = cv2.getOptimalNewCameraMatrix(cameraMatrixL, distL, (widthL, heightL), 1, (widthL, heightL))
np.savetxt('newcameraMatrixL.csv', newCameraMatrixL, delimiter=",")

retR, cameraMatrixR, distR, rvecsR, tvecsR = cv2.calibrateCamera(objPoints, imgPointsR, frameSize, None, None)
np.savetxt('cameraMatrixR.csv', cameraMatrixR, delimiter=",")
heightR, widthR, channelsR = frameR.shape
newCameraMatrixR, roi_R = cv2.getOptimalNewCameraMatrix(cameraMatrixR, distR, (widthR, heightR), 1, (widthR, heightR))
np.savetxt('newcameraMatrixR.csv', newCameraMatrixR, delimiter=",")
# stereo vision calibration

flags = 0
flags |= cv2.CALIB_FIX_INTRINSIC    # fix the intrinsic camera matrix so that we only calculate Rot, Trans, Emat and Fmat, therefore intrinsic parameters are the same

criteria_stereo = (cv2.TERM_CRITERIA_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001)

# this step is performed to transformation between the two cameras and calculate Essential nd Fundamental matrix
retStereo, newCameraMatrixL, distL, newCameraMatrixR, distR, rot, trans, essentialMatrix, fundamentalMatrix = cv2.stereoCalibrate(objPoints, imgPointsL, imgPointsR, newCameraMatrixL, distL,
                                                                                                                        newCameraMatrixR, distR, grayL.shape[::-1], criteria_stereo, flags)
rectifyScale = 1        # as we will only see 1 dimension, we need out other two dimension to be rectified
rectL, rectR, projMatrixL, projMatrixR, Q, roi_L, roi_R = cv2.stereoRectify(newCameraMatrixL, distL, newCameraMatrixR, distR, grayL.shape[::-1], rot, trans, rectifyScale, (0, 0))

stereoMapL = cv2.initUndistortRectifyMap(newCameraMatrixL, distL, rectL, projMatrixL, grayL.shape[::-1], cv2.CV_16SC2)
stereoMapR = cv2.initUndistortRectifyMap(newCameraMatrixR, distR, rectR, projMatrixR, grayR.shape[::-1], cv2.CV_16SC2)

print("Saving Parameters")
cv_file = cv2.FileStorage('stereoMap.xml', cv2.FILE_STORAGE_WRITE)

cv_file.write('stereoMapL_x', stereoMapL[0])
cv_file.write('stereoMapL_y', stereoMapL[1])
cv_file.write('stereoMapR_x', stereoMapL[0])
cv_file.write('stereoMapR_y', stereoMapL[1])

cv_file.release()

