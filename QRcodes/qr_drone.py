import queue
import rospy
from clover import srv
from std_srvs.srv import Trigger
from pyzbar import pyzbar
import numpy as np
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import math


bridge = CvBridge()

rospy.init_node('solvePnP')

color_debug = rospy.Publisher("/color_debug", Image, queue_size=1)

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)


def navigate_wait(x=0, y=0, z=1, yaw=float('nan'), speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if telem.x ** 2 + telem.y ** 2 + telem.z ** 2 < tolerance ** 2:
            break
        rospy.sleep(0.2)

def getCameraInfo():
    info = rospy.wait_for_message('/main_camera/camera_info', CameraInfo)
    cameraMatrix = np.reshape(np.array(info.K, dtype="float64"), (3, 3))
    distortion = np.array(info.D, dtype="float64")
    return cameraMatrix, distortion

n = 0
def image_callback(data):
    global n
    n += 1
    if n >= 5:
        img = bridge.imgmsg_to_cv2(data, 'bgr8')

        barcodes = pyzbar.decode(img)
        for barcode in barcodes:
            b_data = barcode.data.decode('utf-8')
            b_type = barcode.type
            (x, y, w, h) = barcode.rect
            xc = x + w / 2
            yc = y + h / 2
            print('Found {} with data {} with center at x={}, y={}'.format(b_type, b_data, xc, yc))

        

        color_debug.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))
        n = 0

try:
    rospy.Subscriber('/main_camera/image_raw', Image, image_callback)
    rospy.spin()


except KeyboardInterrupt:   
    color_debug.unregister()
    print('ending the program')
