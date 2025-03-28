import rospy
from clover import srv
from std_srvs.srv import Trigger

import numpy as np
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import math

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Quaternion
import tf.transformations as t
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import Pose, Point, Quaternion
import tf.transformations as t
import tf2_ros
import tf2_geometry_msgs

import time

import threading

from mavros_msgs.srv import CommandBool, CommandLong
from pymavlink import mavutil

from web_backend import web
import multiprocessing
import time
from random import randint, uniform
from colorama import Fore, Style

if __name__ == '__main__':
    multiprocessing.freeze_support()

    manager = multiprocessing.Manager()
    button_flags = manager.dict()
    marks = manager.list()

def main(button_flags, marks):

    bridge = CvBridge()

    rospy.init_node('solvePnP')

    send_command = rospy.ServiceProxy('mavros/cmd/command', CommandLong)

    color_debug = rospy.Publisher("/color_debug", Image, queue_size=1)
    mask_debug = rospy.Publisher("/mask_debug", Image, queue_size=1)
    markers_arr_pub = rospy.Publisher("/l22_aero_color/markers_viz", MarkerArray)

    get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
    navigate = rospy.ServiceProxy('navigate', srv.Navigate)
    land = rospy.ServiceProxy('land', Trigger)


    # navigation with code blocking
    def navigate_wait(x=0, y=0, z=1, yaw=float('nan'), speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
        if not button_flags['killswitch']:
            navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

        while not rospy.is_shutdown():
            if button_flags['killswitch']:
                send_command(command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, param1=0, param2=21196)
                break

            telem = get_telemetry(frame_id='navigate_target')
            if telem.x ** 2 + telem.y ** 2 + telem.z ** 2 < tolerance ** 2:
                break
            rospy.sleep(0.2)

    def send_markers():
        result = []
        iddd = 0
        for m in markers:
            # result.append(transform_marker(marker, frame_to="aruco_map"))
            # cx_map, cy_map, cz_map, _ = transform_xyz_yaw(
            # marker.cx_cam, marker.cy_cam, marker.cz_cam, 0, "main_camera_optical", frame_to, listener)
            marker = Marker()
            marker.header.frame_id = "aruco_map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "color_markers"
            marker.id = iddd
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = m[2][0]
            marker.pose.position.y = m[2][1]
            marker.pose.position.z = m[2][2]
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.05

            marker.color.a = 0.8

            marker.color.r = rgb[m[0]][0]
            marker.color.g = rgb[m[0]][1]
            marker.color.b = rgb[m[0]][2]
            result.append(marker)
            iddd += 1

        markers_arr_pub.publish(MarkerArray(markers=result))

    rgb = {
        'blue': (0, 0, 255),
        'green': (0, 255, 0),
        'red': (255, 0, 0),
        'yellow': (255, 255, 0),
        'orange': (255, 155, 0),


    }

    # def terminal_debug(cords):
    #     cords = list(set(cords))
    #     for sign, num in zip(cords, range(1, len(cords) + 1)):
    #         num = str(num)
    #         if sign[0] == 'blue':
    #             print(f'{Style.BRIGHT + Fore.WHITE + num}) Color: {Fore.BLUE + str(sign[0])}{Fore.WHITE}, Type: {Fore.MAGENTA +str(sign[4])}\n\
    #   {Fore.WHITE}x: {Style.BRIGHT + str(sign[1])}\n\
    #   y: {Style.BRIGHT + str(sign[2])}\n\
    #   z: {Style.BRIGHT + str(sign[3])}')
    #         elif sign[0] == 'yellow':
    #             print(f'{Style.BRIGHT + Fore.WHITE + num}) Color: {Fore.YELLOW + str(sign[0])}{Fore.WHITE}, Type: {Fore.MAGENTA +str(sign[4])}\n\
    #   {Fore.WHITE}x: {Style.BRIGHT + str(sign[1])}\n\
    #   y: {Style.BRIGHT + str(sign[2])}\n\
    #   z: {Style.BRIGHT + str(sign[3])}')
    #         elif sign[0] == 'red':
    #             print(f'{Style.BRIGHT + Fore.WHITE + num}) Color: {Fore.RED + str(sign[0])}{Fore.WHITE}, Type: {Fore.MAGENTA +str(sign[4])}\n\
    #   {Fore.WHITE}x: {Style.BRIGHT + str(sign[1])}\n\
    #   y: {Style.BRIGHT + str(sign[2])}\n\
    #   z: {Style.BRIGHT + str(sign[3])}')


    # # drawing corners for debugging
    # def drawCorners(img, points):
    #     for i, p in enumerate(points):
    #         p = np.int0(p)
    #         x, y = p[0], p[1]
    #         font = cv2.FONT_HERSHEY_SIMPLEX
    #         fontScale = 0.4
    #         color = (255, 255, 255)
    #         color_bg = (255, 0, 255)
    #         thickness = 1
    #         text = str(i + 1)
    #         text_size, _ = cv2.getTextSize(text, font, fontScale, thickness)
    #         w, h = text_size
    #         cv2.circle(img,(x, y), h // 2, color_bg, -1)
    #         cv2.putText(img, text, (x - w // 2, y + h // 2), font, fontScale, color, thickness)


    # get camera matrix and distortion coefficients
    def getCameraInfo():
        info = rospy.wait_for_message('/main_camera/camera_info', CameraInfo)
        cameraMatrix = np.reshape(np.array(info.K, dtype="float64"), (3, 3))
        distortion = np.array(info.D, dtype="float64")
        return cameraMatrix, distortion


    # the coordinates of the corners of the markers in their coordinate system
    def getObjectPoints(squareLength):
        objectPoints = [
            [-squareLength / 2, -squareLength / 2, 0],
            [squareLength / 2, -squareLength / 2, 0],
            [squareLength / 2, squareLength / 2, 0],
            [-squareLength / 2, squareLength / 2, 0],
        ]
        objectPoints = np.array(objectPoints, dtype="float64")
        return objectPoints


    # getting the coordinates of marker corners in pixels
    def getImagePoints(cnt):
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        return box


    # coordinate transformation function
    def transformPose(inputPose, fromFrame, toFrame):
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)

        poseStamped = tf2_geometry_msgs.PoseStamped()
        poseStamped.pose = inputPose
        poseStamped.header.frame_id = fromFrame
        poseStamped.header.stamp = rospy.Time()

        try:
            transformed = tfBuffer.transform(poseStamped, toFrame, rospy.Duration(1))
            return transformed.pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise


    # get pose of the found objects
    def getPose(cnt, size):
        objectPoints = getObjectPoints(size)

        imagePoints = getImagePoints(cnt)

        cameraMatrix, distortion = getCameraInfo()

        retval, rvec, tvec = cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, distortion)

        pose = Pose()
        pose.position = Point(*tvec)
        pose.orientation = Quaternion(*t.quaternion_from_euler(rvec[0][0], rvec[1][0], rvec[2][0]))

        transformed = transformPose(pose, "main_camera_optical", "aruco_map")
        x, y, z = transformed.position.x, transformed.position.y, transformed.position.z
        # x, y, z = (0,0,0)
        # euler = t.euler_from_quaternion((transformed.orientation.x, transformed.orientation.y, transformed.orientation.z, transformed.orientation.w))

        # pose = (round(x, 3), round(y, 3), round(z, 3))
        # for marker in markers:
        #     if math.sqrt((pose[0] - marker[2][0])**2 + (pose[1] - marker[2][1])**2 + (pose[2] - marker[2][2])**2) < 0.5:
        #         break
        # else:
        #     markers.append(["color", "shape", pose])

        return round(x, 3), round(y, 3), round(z, 3)


    def get_shape(cnt):
        epsilon = 0.1 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)

        # check for touch to boarding
        for c in approx:
            # print((c[0][0], c[0][1]))
            if c[0][0] == 0 or c[0][0] == 319 or c[0][1] == 0 or c[0][1] == 239:
                return False

        for i, shape in enumerate(['triangle', 'square', 'pentagon', 'hexagon', 'octagon'], 3):
            if i == len(approx):
                return shape
            if len(approx) > 8:
                return "circle"


    def draw_marker(image, cnt, shape):
        cv2.drawContours(image, [cnt], -1, (0, 0, 0), 3)

        M = cv2.moments(cnt)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        image = cv2.putText(image, shape, (cx - 50, cy), cv2.FONT_HERSHEY_SIMPLEX, 1, (227, 0, 227), 2, cv2.LINE_AA)


    # main function
    # n = 0
    def image_callback():
        # def image_callback(img):
        # global n
        # n += 1
        # if n >= 5:
        #     n = 0

        while True:
            start_time = time.time()
            img = rospy.wait_for_message('/main_camera/image_raw_throttled', Image)
            img = bridge.imgmsg_to_cv2(img, 'bgr8')
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            mask_all = []
            for color in ranges:
                mask = cv2.inRange(hsv, ranges[color][0], ranges[color][1])
                if len(mask_all):
                    mask_all += mask
                else:
                    mask_all = mask
                kernel = np.ones((5, 5), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

                contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
                contours = [cont for cont in contours if cv2.contourArea(cont) > 500]

                for cnt in contours:
                    M = cv2.moments(cnt)
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    cv2.circle(img, (cx, cy), 7, (0, 0, 255), -1)

                    if 80 < cx < 240 and 40 < cy < 200:
                        shape = get_shape(cnt)
                        if shape:
                            draw_marker(img, cnt, shape)

                            # threading.Thread(target=getPose, args=(cnt, 0.5), daemon=True).start()

                            pose = getPose(cnt, 0.4)
                            # pose = (0,0,0)

                            for marker in markers:
                                if math.sqrt((pose[0] - marker[2][0]) ** 2 + (pose[1] - marker[2][1]) ** 2 + (
                                        pose[2] - marker[2][2]) ** 2) < 0.5:
                                    break
                            else:
                                markers.append([color, shape, pose])
                                marks.append([pose[0], pose[1], color, shape])

            duration = time.time() - start_time
            print("--- %s seconds ---" % (round(duration, 2)))

            # contours output in topic color_debug
            mask_all = cv2.cvtColor(mask_all, cv2.COLOR_GRAY2BGR)
            mask_debug.publish(bridge.cv2_to_imgmsg(mask_all, 'bgr8'))
            img = cv2.rectangle(img, (80, 40), (240, 200), (0, 0, 255), 1)
            color_debug.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))
            send_markers()


    # Constants

    markers = []

    # real hsv
    ranges = {
        'yellow': ((25, 50, 100), (35, 255, 255)),
        'blue': ((90, 35, 130), (130, 255, 255)),
        # 'red': ((160, 50, 100), (180, 255, 255)),
        'orange': ((0, 50, 150), (24, 255, 255)),
    }

    # hsv for sim
    # ranges = {
    #     'yellow': ((20, 100, 100), (40, 255, 255)),
    #     'blue': ((110, 100, 100), (130, 255, 255)),
    #     'red': ((0, 100, 100), (10, 255, 255)),
    #     'green': ((60, 100, 100), (80, 255, 255)),
    # }

    print('program started')
    while True:
        print('while started')
        while not button_flags['start']:
            pass
        for i in range(len(marks)):
            marks.pop()
        print('armed')

        # Takeoff and flight to the central point
        navigate_wait(frame_id='body', auto_arm=True)

        # rospy.Subscriber('/main_camera/image_raw_throttled', Image, image_callback)
        threading.Thread(target=image_callback, daemon=True).start()

        # Flight
        for y in range(3):
            for x in range(3):
                if button_flags['stop'] or button_flags['killswitch']:
                    break
                while button_flags['pause']:
                    pass
                navigate_wait(x=x, y=y, z=1, frame_id='aruco_map')

            if button_flags['stop'] or button_flags['killswitch']:
                break

        # Return to the zero point and landing
        navigate_wait(frame_id='aruco_map')
        # terminal_debug(crds)
        print(markers)
        print(len(markers))
        land()


        button_flags['start'] = False
        button_flags['stop'] = True


if __name__ == '__main__':
    button_flags["start"] = False
    button_flags["stop"] = True
    button_flags["pause"] = False
    button_flags["killswitch"] = False

    main_thread = multiprocessing.Process(target=main, args=(button_flags, marks))
    main_thread.start()

    try:
        web.button_flags = button_flags
        web.marks = marks
        web.app.run(host='192.168.50.50', port=5000, debug=False)
    except Exception as e:
        print(f'error: {e}')
    finally:
        main_thread.terminate()
        main_thread.join()
