# coding=utf-8
from urllib.request import urlopen
import cv2
import numpy as np


def nothing(*args):
    pass

cap = cv2.VideoCapture('http://192.168.11.1:8080/stream?topic=/main_camera/image_raw')
# cap = cv2.VideoCapture(0)

# cv2.namedWindow("result")
cv2.namedWindow("range_1", cv2.WINDOW_GUI_NORMAL)
cv2.namedWindow("range_2", cv2.WINDOW_GUI_NORMAL)
cv2.resizeWindow('range_1', 400, 100)
cv2.resizeWindow('range_2', 400, 100)

cv2.createTrackbar('h_min', 'range_1', 0, 179, nothing)
cv2.createTrackbar('s_min', 'range_1', 0, 255, nothing)
cv2.createTrackbar('v_min', 'range_1', 0, 255, nothing)
cv2.createTrackbar('h_max', 'range_1', 179, 179, nothing)
cv2.createTrackbar('s_max', 'range_1', 255, 255, nothing)
cv2.createTrackbar('v_max', 'range_1', 255, 255, nothing)
# cv2.createTrackbar('null', 'range_1', 255, 255, nothing)

cv2.createTrackbar('h_min', 'range_2', 0, 179, nothing)
cv2.createTrackbar('s_min', 'range_2', 0, 255, nothing)
cv2.createTrackbar('v_min', 'range_2', 0, 255, nothing)
cv2.createTrackbar('h_max', 'range_2', 0, 179, nothing)
cv2.createTrackbar('s_max', 'range_2', 0, 255, nothing)
cv2.createTrackbar('v_max', 'range_2', 0, 255, nothing)
# cv2.createTrackbar('null', 'range_2', 255, 255, nothing)

try:

    while True:
        ret, img = cap.read()

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # считываем значения бегунков
        h_min_1 = cv2.getTrackbarPos('h_min', 'range_1')
        s_min_1 = cv2.getTrackbarPos('s_min', 'range_1')
        v_min_1 = cv2.getTrackbarPos('v_min', 'range_1')
        h_max_1 = cv2.getTrackbarPos('h_max', 'range_1')
        s_max_1 = cv2.getTrackbarPos('s_max', 'range_1')
        v_max_1 = cv2.getTrackbarPos('v_max', 'range_1')
        h_min_2 = cv2.getTrackbarPos('h_min', 'range_2')
        s_min_2 = cv2.getTrackbarPos('s_min', 'range_2')
        v_min_2 = cv2.getTrackbarPos('v_min', 'range_2')
        h_max_2 = cv2.getTrackbarPos('h_max', 'range_2')
        s_max_2 = cv2.getTrackbarPos('s_max', 'range_2')
        v_max_2 = cv2.getTrackbarPos('v_max', 'range_2')

        # формируем начальный и конечный цвет фильтра
        min1 = np.array((h_min_1, s_min_1, v_min_1), np.uint8)
        max1 = np.array((h_max_1, s_max_1, v_max_1), np.uint8)

        min2 = np.array((h_min_2, s_min_2, v_min_2), np.uint8)
        max2 = np.array((h_max_2, s_max_2, v_max_2), np.uint8)

        # накладываем фильтр на кадр в модели HSV
        kernel = np.ones((2, 2), np.uint8)

        thresh1 = cv2.inRange(hsv, min1, max1)
        # thresh1 = cv2.morphologyEx(thresh1, cv2.MORPH_OPEN, kernel)

        thresh2 = cv2.inRange(hsv, min2, max2)
        # thresh2 = cv2.morphologyEx(thresh2, cv2.MORPH_OPEN, kernel)

        cv2.imshow('result', thresh1 + thresh2)
        cv2.imshow('img', img)

        cv2.moveWindow('img', 790, 50)
        cv2.moveWindow('result', 790, 350)
        cv2.moveWindow('range_1', 100, 0)
        cv2.moveWindow('range_2', 100, 400)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            raise KeyboardInterrupt
            break

except KeyboardInterrupt:
    print(f'Min mask: ({h_min_1}, {s_min_1}, {v_min_1})')
    print(f'Max mask: ({h_max_1}, {s_max_1}, {v_max_1})')
    print(f'Min mask2: ({h_min_2}, {s_min_2}, {v_min_2})')
    print(f'Max mask2: ({h_max_2}, {s_max_2}, {v_max_2})')

cap.release()
cv2.destroyAllWindows()
