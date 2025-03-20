from threading import Thread
from ultralytics import YOLO
import cv2
import threading

yolo = YOLO('best.pt')  # insert the name of your model here
cap = cv2.VideoCapture("http://192.168.11.1:8080/stream?topic=/main_camera/image_raw")

ret, img = cap.read()

def capture_frames():
    global cap
    global img
    while True:
        ret, img = cap.read()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


cap_thread = threading.Thread(target=capture_frames)
cap_thread.start()

while True:
    results = yolo.track(img, stream=True)

    for i in results:
        names = i.names

        for j in i.boxes:
            if j.conf > 0.1:
                [x1, y1, x2, y2] = j.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                _class = int(j.cls[0])
                _class_name = names[_class]

                cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(img, f'{names[int(j.cls[0])]} {j.conf[0]:.2f}', (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow('frame', img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
