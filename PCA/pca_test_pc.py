import cv2
import numpy as np

lower = (52, 89, 63)
upper = (179, 255, 255)

try:
    cap = cv2.VideoCapture(0)

    while True:
        _, img = cap.read()


        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        mask = cv2.inRange(hsv, lower, upper)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


        if contours: 
            contours = sorted(list(contours), key=lambda x: cv2.contourArea(x), reverse=True)
            cv2.drawContours(img, contours[1::], -1, (255, 255, 255), 3, cv2.LINE_AA)
            cv2.drawContours(img, contours, 0, (0, 255, 0), 3, cv2.LINE_AA)
            largest = contours[0]
            cv2.drawContours(img, largest, -1, (255, 0, 0), 3, cv2.LINE_AA)
            # for i in contours:
            #     x, y, w, h = cv2.boundingRect(i)
            #     cv2.putText(img, f'{cv2.contourArea}')
            largest = largest.reshape(-1, 2).astype(np.float32)
            
            if len(largest) > 2:
                mean, eig = cv2.PCACompute(largest, mean=None)

                dir = eig[0]
                angle = np.degrees(np.arctan2(-dir[1], dir[0]))

                center = tuple([int(i) for i in mean[0]])
                end = (int(center[0] + dir[0] * 100), int(center[1] + dir[1] * 100))

                cv2.circle(img, center, 5, (0, 0, 255), -1)
                cv2.arrowedLine(img, center, end, (0, 0, 255), 2)



        cv2.imshow('frame', img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            raise KeyboardInterrupt
        
except KeyboardInterrupt:
    print('user pressed Q. Finishing the program.')
