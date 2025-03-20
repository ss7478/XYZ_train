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
from colorama import Fore, Style
from mavros_msgs.srv import CommandBool, CommandLong
from pymavlink import mavutil

from web_backend import web
import multiprocessing
import time
from random import randint, uniform


if __name__ == '__main__':
    multiprocessing.freeze_support()
    
    manager = multiprocessing.Manager()
    button_flags = manager.dict()
    marks = manager.list() 



def main(button_flags, marks):
    markers_arr_pub = rospy.Publisher("/l22_aero_color/markers_viz", MarkerArray)


    def send_markers():
        result = []
        iddd = 0
        for crd in crds:
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
            marker.pose.position.x = crd[1]
            marker.pose.position.y = crd[2]
            marker.pose.position.z = crd[3]
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.05

            marker.color.a = 0.8

            marker.color.r = rgb[crd[0]][0]
            marker.color.g = rgb[crd[0]][1]
            marker.color.b = rgb[crd[0]][2]
            result.append(marker)
            iddd += 1
        markers_arr_pub.publish(MarkerArray(markers=result))


    bridge = CvBridge()

    rospy.init_node('solvePnP')

    color_debug = rospy.Publisher("/color_debug", Image)

    get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
    navigate = rospy.ServiceProxy('navigate', srv.Navigate)
    land = rospy.ServiceProxy('land', Trigger)

    send_command = rospy.ServiceProxy('mavros/cmd/command', CommandLong)

    # empty coordinates list
    crds = []

    # hsv for real drone
    colors = {
        'green': [((24,49,90), (61,173,255)), (0, 255,255)],
        'blue': [((68,56,173), (150,255,255)), (255 , 0, 0)],
        'red': [((138,44,189), (179,255,255)), (0, 0, 255)]
    }

    # # hsv for sim
    # colors = {
    #     'blue': [((120, 100, 100), (120, 255, 255)), (255, 0, 0)],
    #     'green': [((71, 110, 150), (110, 255, 255)), (255, 255, 0)],
    #     'red': [((0, 100, 100), (10, 255, 255)), (0, 0, 255)]
    # }
    
    cl = {
        'blue': 0,
        'green': 0,
        'red': 0
    }
    rgb = {
        'blue': (0, 0, 255),
        'green': (0, 255, 0),
        'red': (255, 0, 0),
    }


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
    def getImagePoints(hsv, range, color):
        fig = ['Triangle', 'square', 'pentagon', 'hexagon', 'octagon']

        

        kernel = np.ones((11, 11), np.uint8)
        mask = cv2.inRange(hsv, range[0], range[1])
        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
        contours, hierarchy = cv2.findContours(closing.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[:2]
        contours = [cont for cont in contours if cv2.contourArea(cont) > 200]
        cl[color] = contours
        boxes = []
        boxes_fig = []

        for countt, cnt in enumerate(contours):
            flag = False
            approx = cv2.approxPolyDP(cnt, 12, True)
            M = cv2.moments(cnt)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            for c in cnt:
                # print((c[0][0], c[0][1]))
                if c[0][0] == 0 or c[0][0] == 319 or c[0][1] == 0 or c[0][1] == 239:
                    flag = True
                    break
            if flag:
                contours.pop(countt)
            else:
                rect = cv2.minAreaRect(cnt)
                cent = rect[1]

                if distanceCalculate(cent) <= 150:
                    boxes.append(cv2.boxPoints(rect))
                    for count, a in enumerate(fig, 3):
                        if count == len(approx):
                            boxes_fig.append(a)
                        if len(approx) > 8:
                            boxes_fig.append('circle')

        return boxes, boxes_fig, mask, contours


    # function for crating virtual crop
    def distanceCalculate(cent):
        arr = list(cent)
        """p1 and p2 in format (x1,y1) and (x2,y2) tuples"""
        dis = ((160 - cent[0]) ** 2 + (120 - cent[1]) ** 2) ** 0.5
        return dis


    def terminal_debug(cords):
        for sign, num in zip(cords, range(1, len(cords) + 1)):
            num = str(num)
            if sign[0] == 'green':
                print(
                    f'{Style.BRIGHT + Fore.WHITE + num}) Color: {Fore.BLUE + str(sign[0])}{Fore.WHITE}, Type: {Fore.MAGENTA + str(sign[4])}\n\
    {Fore.WHITE}x: {Style.BRIGHT + str(sign[1])}\n\
    y: {Style.BRIGHT + str(sign[2])}\n\
    z: {Style.BRIGHT + str(sign[3])}')
            elif sign[0] == 'blue':
                print(
                    f'{Style.BRIGHT + Fore.WHITE + num}) Color: {Fore.YELLOW + str(sign[0])}{Fore.WHITE}, Type: {Fore.MAGENTA + str(sign[4])}\n\
    {Fore.WHITE}x: {Style.BRIGHT + str(sign[1])}\n\
    y: {Style.BRIGHT + str(sign[2])}\n\
    z: {Style.BRIGHT + str(sign[3])}')
            elif sign[0] == 'red':
                print(
                    f'{Style.BRIGHT + Fore.WHITE + num}) Color: {Fore.RED + str(sign[0])}{Fore.WHITE}, Type: {Fore.MAGENTA + str(sign[4])}\n\
    {Fore.WHITE}x: {Style.BRIGHT + str(sign[1])}\n\
    y: {Style.BRIGHT + str(sign[2])}\n\
    z: {Style.BRIGHT + str(sign[3])}')


    # drawing corners for debugging
    def drawCorners(img, points):
        for i, p in enumerate(points):
            p = np.int0(p)
            x, y = p[0], p[1]
            font = cv2.FONT_HERSHEY_SIMPLEX
            fontScale = 0.4
            color = (255, 255, 255)
            color_bg = (255, 0, 255)
            thickness = 1
            text = str(i + 1)
            text_size, _ = cv2.getTextSize(text, font, fontScale, thickness)
            w, h = text_size
            cv2.circle(img, (x, y), h // 2, color_bg, -1)
            cv2.putText(img, text, (x - w // 2, y + h // 2), font, fontScale, color, thickness)


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
    def getPose(color, objectPoints, imagePoints, cameraMatrix, distortion, boxes_fig, count):
        retval, rvec, tvec = cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, distortion)



        pose = Pose()
        pose.position = Point(*tvec)
        pose.orientation = Quaternion(*t.quaternion_from_euler(rvec[0][0], rvec[1][0], rvec[2][0]))

        transformed = transformPose(pose, "main_camera_optical", "aruco_map")
        x, y, z = transformed.position.x, transformed.position.y, transformed.position.z
        euler = t.euler_from_quaternion(
            (transformed.orientation.x, transformed.orientation.y, transformed.orientation.z, transformed.orientation.w))

        return round(x, 1), round(y, 1), round(z, 1), boxes_fig[count]


    def drawing(image, c_list, contours, cl):
        fig = ['Triangle', 'square', 'pentagon', 'hexagon', 'octagon']
        kernel = np.ones((11, 11), np.uint8)
        for_debug = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        opening = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
        closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
        contours, hierarchy = cv2.findContours(closing, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 200]
        contours.sort(key=lambda x: cv2.contourArea(x), reverse=True)
        poly = []
        for color in c_list:
            for cnt in cl[color]:
                approx = cv2.approxPolyDP(cnt, 12, True)
                poly.append(approx)
                M = cv2.moments(cnt)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                for count, a in enumerate(fig, 3):
                    if count == len(approx):
                        for_debug = cv2.putText(for_debug, a, (cx - 50, cy), cv2.FONT_HERSHEY_SIMPLEX, 1, (227, 0, 227), 2,
                                                cv2.LINE_AA)
                    if len(approx) > 8:
                        for_debug = cv2.putText(for_debug, 'circle', (cx - 50, cy), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                                (227, 0, 227), 2, cv2.LINE_AA)

            cv2.drawContours(for_debug, poly, -1, colors[color][1], 3)
            poly.clear()
        color_debug.publish(bridge.cv2_to_imgmsg(for_debug, 'bgr8'))


    # main function
    def image_callback(data):
        mask_all = []
        # img = rospy.wait_for_message('/main_camera/image_raw_throttled',Image)
        img = bridge.imgmsg_to_cv2(data, 'bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        for color in ['blue', 'green', 'red']:
            objectPoints = getObjectPoints(0.5)
            boxes, boxes_fig, mask, contours = getImagePoints(hsv, colors[color][0], color)
            cameraMatrix, distortion = getCameraInfo()
            mask_all.append(mask)
            for count, imagePoints in enumerate(boxes):
                c = list(getPose(color, objectPoints, imagePoints, cameraMatrix, distortion, boxes_fig, count))

                for ck, crd in enumerate(crds):
                    if math.sqrt((c[0] - crd[1]) ** 2 + (c[1] - crd[2]) ** 2 + (c[2] - crd[3]) ** 2) < 0.6:
                        break
                else:
                    crds.append([color, c[0], c[1], c[2], c[3]])
        send_markers()

        # contours output in topic color_debug
        im = mask_all[0] + mask_all[1] + mask_all[2]
        drawing(im, ['blue', 'green', 'red'], colors, cl)

    print('program started')
    while True:
        print('while started')
        while not button_flags['start']:
            pass
        for i in range(len(marks)):
            marks.pop()
        print('armed')
        navigate_wait(frame_id='body', auto_arm=True)

        suid = rospy.Subscriber('/main_camera/image_raw_throttled', Image, image_callback)
        rospy.sleep(1)

        # fly
        for y in range(3):
            for x in range(3):
                if button_flags['stop'] or button_flags['killswitch']:
                    break
                while button_flags['pause']:
                    pass
                navigate_wait(x=x, y=y, z=1, frame_id='aruco_map')
                    
            if button_flags['stop'] or button_flags['killswitch']:
                break

        # return to the zero point and landing
        suid.unregister()

        terminal_debug(crds)

        navigate_wait(x=0, y=0, z=1, frame_id='aruco_map')
        print('landed')
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
        web.app.run(host='192.168.11.1', port=5000, debug=False)
    except Exception as e:
        print(f'error: {e}')
    finally:
        main_thread.terminate()
        main_thread.join()
