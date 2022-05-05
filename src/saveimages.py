import cv2

cap = cv2.VideoCapture(0)
cap2 = cv2.VideoCapture(2)

num = 0
l_camera = 'C:\\Users\\HP\\PycharmProjects\\RRTPathPlanning\\venv\\L_Camera\\'
r_camera = 'C:\\Users\\HP\PycharmProjects\\RRTPathPlanning\\venv\R_Camera\\'

while cap.isOpened():
    suc1, img = cap.read()
    suc2, img2 = cap2.read()
    key = cv2.waitKey(1)

    if key == 27:
        break
    elif key == ord('s'):
        cv2.imwrite(l_camera + str(num) + '.png', img)
        cv2.imwrite(r_camera + str(num) + '.png', img2)
        print("saved")
        num += 1

    cv2.imshow('output1', img)
    cv2.imshow('output2', img2)
