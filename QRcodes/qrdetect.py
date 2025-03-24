import cv2

class qrdet:
    def __init__(self, cap = cv2.VideoCapture(0)):    # pass cv2.videocapture object here
        self.qr = cv2.QRCodeDetector()
        self.cap = cap
        self.ret = False
        self.data = ()
        self.points = []
    

    def detect(self, img):     #pass frame object here
        self.img = img
        self.ret, self.data, self.points, _ = self.qr.detectAndDecodeMulti(self.img)
        return self.ret
    
    def getdata(self):
        return self.data
    
    def getpoints(self):
        return self.points
    
    def drawall(self, img = None):
        if img is None:
            img = self.img
        if self.points is not None:
            return cv2.polylines(img, self.points.astype(int), True, (0, 255, 0), 3)
        return img


try:
    cap = cv2.VideoCapture(0)
    qr = qrdet(cap)

    while True:
        _, img = cap.read()
        qr.detect(img)
        img = qr.drawall(img)
        print(qr.getdata())

        cv2.imshow('frame', img)

        if cv2.waitKey(1) == ord('q'):
            raise KeyboardInterrupt
            break

except KeyboardInterrupt:
    print('program finished cause user closed the window')
    cap.release()
    cv2.destroyAllWindows()
