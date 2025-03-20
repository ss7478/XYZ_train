import cv2

cap = cv2.VideoCapture("http://192.168.11.1:8080/stream?topic=/main_camera/image_raw")
cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
name = 38
while True:
    ret, img = cap.read()
    cv2.imshow('frame', img)
    key = cv2.waitKey(10) & 0xFF
    if key == ord('g'):
        new_name = str(name) + '.jpeg'
        cv2.imwrite(new_name, img)
        name += 1
        print('captured a photo')

    if key == ord('q'):
        break
