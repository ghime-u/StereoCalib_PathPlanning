import cv2

# un-distortion and rectification

cv_file = cv2.FileStorage
cv_file.open('stereoMap.xml', cv2.FileStorage_READ)

stereoMapL_x = cv_file.getNode('stereoMapL_x').mat()
stereoMapL_y = cv_file.getNode('stereoMapL_y').mat()
stereoMapR_x = cv_file.getNode('stereoMapL_x').mat()
stereoMapR_y = cv_file.getNode('stereoMapL_y').mat()


def undistortRectify(frameR, frameL):

    #undistort and rectify
    undisstortedL = cv2.remap(frameL, stereoMapL_x, stereoMapL_y)
    undisstortedR = cv2.remap(frameR, stereoMapR_x, stereoMapR_y)

    return undisstortedR, undisstortedL