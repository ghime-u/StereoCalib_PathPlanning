import cv2
import numpy as np
import os
import sys
import glob
import imutils
import Triangulation as tri
import csv


# parameters

distance_camera = 28 # distance between stereo setup in cm


def get_viewx(csvfile, img):
    with open(csvfile, newline='') as csv_file:
        reader = csv.reader(csv_file, delimiter=',')
        cm = list(reader)
    focalx = float(cm[0][0])
    focaly = float(cm[1][1])
    w = img.shape[1]
    temp = w/(2*focalx)
    angle_x = 2*57.89*np.arctan(temp)
    return angle_x


capL = cv2.VideoCapture(0, cv2.CAP_DSHOW)
capR = cv2.VideoCapture(6, cv2.CAP_DSHOW)


def shapeRecognition(frame, mask):
    contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    center = None
    copy = frame.copy()
    # if contours are found

    for cnt in contours:
        center = []
        area = cv2.contourArea(cnt)
        if area > 300:
            cv2.drawContours(copy, cnt, -1, (255, 0, 0), 3)
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            x, y, w, h = cv2.boundingRect(approx)

            cv2.rectangle(copy, (x, y), (x + w, y + h), (0, 0, 0), 4)
            aspect_ratio = w / float(h)
            objectcor = len(approx)
            if objectcor == 4:
                print("rectangle")
                center_cord = [x + (w/2), y + (h/2)]
                pos_cord = [x, w]
                center.append(center_cord)
                return [center_cord, pos_cord]



def get_contours(img, imgcopy):
    contour, heirarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contour:
        x, y, w, h = cv2.boundingRect(cnt)
        print(x, y, w, h)
    cv2.drawContours(imgcopy, contour, -1, (255, 255, 255), 3)


def color_filter(image, camera):
    lower_blue = np.array([0, 70, 50])
    upper_blue =  np.array([10, 255, 255])
    blur = cv2.bilateralFilter(image, 9, 75, 75)
    imgHsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    if camera == 1:
        mask = cv2.inRange(imgHsv, lower_blue, upper_blue)
    if camera == 0:
        mask = cv2.inRange(imgHsv, lower_blue, upper_blue)

    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    return mask


def main():
    count = -1;
    obspos = []
    while True:
        depth = []
        count +=1
        suc1, imgL = capL.read()
        suc2, imgR = capR.read()
        cameramatrixR = 'cameraMatrixR.csv'
        angle_Rx = get_viewx(cameramatrixR, imgR)

        if suc1 == False or suc2 == False:
            break
        else:
            maskL = color_filter(imgL, 1)
            maskR = color_filter(imgR, 0)

            frameL = cv2.bitwise_and(imgL, imgL, mask=maskL)
            frameR = cv2.bitwise_and(imgR, imgR, mask=maskR)

            rect_l = shapeRecognition(imgL, maskL)
            rect_r = shapeRecognition(imgR, maskR)

            if np.all(rect_l) == None or np.all(rect_r) == None:
                cv2.putText(imgL, 'Tracking_Lost', (75, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
                cv2.putText(imgR, 'Tracking_Lost', (75, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

            else:
                depth1 = tri.find_depth(rect_r[0], rect_l[0], imgR, imgL, distance_camera, angle_Rx)
                depth.append(depth1)
                obspos.append([depth1, rect_l[1][0], rect_l[1][1], rect_l[1][1]])

                cv2.putText(imgR, "TRACKING", (75, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (124, 252, 0), 2)

                cv2.putText(imgL, "TRACKING", (75, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (124, 252, 0), 2)

                cv2.putText(imgR, "Distance: " + str(round(depth1, 3)), (200, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (124, 252, 0), 2)

                cv2.putText(imgL, "Distance: " + str(round(depth1, 3)), (200, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (124, 252, 0), 2)

                # Multiply computer value with 205.8 to get real-life depth in [cm]. The factor was found manually.

                print("Depth: ", depth)

                # Show the frames

            cv2.imshow("frame right", imgR)

            cv2.imshow("frame left", imgL)

            cv2.imshow("mask right", maskR)

            cv2.imshow("mask left", maskL)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    np.savetxt('position.csv', obspos, delimiter=',')


if __name__ == "__main__":
    main()
