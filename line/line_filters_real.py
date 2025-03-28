import multiprocessing
import time
import rospy
from clover import srv
from std_srvs.srv import Trigger
from pymavlink import mavutil
from mavros_msgs.srv import CommandBool, CommandLong
import numpy as np
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import math
from sensor_msgs.msg import Range

from geometry_msgs.msg import Pose, Point, Quaternion
import tf.transformations as t
import tf2_ros
import tf2_geometry_msgs
import threading

import time

lower = (160, 15, 156)
upper = (179, 255, 255)

bridge = CvBridge()

rospy.init_node('flight', anonymous=True)

color_debug = rospy.Publisher("/color_debug", Image, queue_size=1)

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
set_yaw = rospy.ServiceProxy('set_yaw', srv.SetYaw)
set_yaw_rate = rospy.ServiceProxy('set_yaw_rate', srv.SetYawRate)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)


def navigate_wait(x=0, y=0, z=0.3, yaw=float('nan'), speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if telem.x ** 2 + telem.y ** 2 + telem.z ** 2 < tolerance ** 2:
            break
        rospy.sleep(0.2)


angle = 0
px = 0
py = 0


def aruco_pose():
    navigate_wait(x=0, y=0, z=1, yaw=float('nan'), frame_id='aruco_map')
    land()


flag = True
filters_flag = True


def process_image():
    global angle, px, py, flag, filters_flag
    while flag:
        img = rospy.wait_for_message('/main_camera/image_raw', Image)
        img = bridge.imgmsg_to_cv2(img, 'bgr8')

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, lower, upper)

        # img = img[79:239, 79:159]
        # hsv_crop = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # mask_crop = cv2.inRange(hsv_crop, lower, upper)

        # kernel = np.ones((5,5), np.uint8)
        # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # contours, _ = cv2.findContours(mask_crop, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img, contours, -1, (255, 255, 255), 3, cv2.LINE_AA)

        #         and area_first < area_second)

        if contours:
            contours = [i for i in contours if cv2.contourArea(i) > 40]
            contours = sorted(list(contours), key=lambda x: cv2.contourArea(x), reverse=True)
            contours_area = [cv2.contourArea(i) for i in contours]
            for i, cnt in enumerate(contours):
                cnt_area = cv2.contourArea(cnt)
                bounding_cnt = cv2.boundingRect(cnt)
                if (max(math.fabs(bounding_cnt[3]), math.fabs(bounding_cnt[2])) / min(math.fabs(bounding_cnt[3]),
                                                                                      math.fabs(bounding_cnt[2]))) >= 2 \
                        and cnt_area == contours_area[0]:
                    largest = contours[i]
                    filters_flag = True
                    break
                elif 1 <= (max(math.fabs(bounding_cnt[3]), math.fabs(bounding_cnt[2])) / min(math.fabs(bounding_cnt[3]),
                                                                                             math.fabs(bounding_cnt[
                                                                                                           2]))) <= 1.4:
                    if math.fabs(bounding_cnt[2] * bounding_cnt[3]) / cnt_area >= 2:
                        largest = contours[i]
                        filters_flag = True
                        break
                else:
                    filters_flag = False

            # if (max(math.abs(bounding_cnt[3]), math.abs(bounding_cnt[2]))/ min(math.abs(bounding_cnt[3]), math.abs(bounding_cnt[2]))) >= 2 \
            #      or (1 <= (max(math.abs(bounding_cnt[3]), math.abs(bounding_cnt[2]))/ min(math.abs(bounding_cnt[3]), math.abs(bounding_cnt[2]))) <= 1.4 \
            #         and area_first < area_second):
            if filters_flag == True:
                cv2.drawContours(img, contours[1::], -1, (255, 255, 255), 3, cv2.LINE_AA)
                # cv2.drawContours(img, largest, 0, (0, 255, 0), 3, cv2.LINE_AA)
                cv2.drawContours(img, largest, -1, (255, 0, 0), 3, cv2.LINE_AA)

                largest = largest.reshape(-1, 2).astype(np.float32)

                if len(largest) > 2:
                    mean, eig = cv2.PCACompute(largest, mean=None)

                    dir = eig[0]
                    # print(dir[0], dir[1], end=' ')
                    if dir[1] > 0:
                        dir = -dir
                    # print(dir[0], dir[1])
                    angle = np.degrees(np.arctan2(-dir[1], dir[0]))
                    angle = -angle + 90
                    center = tuple([int(i) for i in mean[0]])
                    # print(angle)
                    cv2.putText(img, f'angle: {round(angle, 1)}', (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255),
                                1, cv2.LINE_AA)
                    cv2.putText(img, f'center: {round(center[0], 1)}', (160, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 0, 255), 1, cv2.LINE_AA)

                    px, py = center[0], center[1]
                    end = (int(center[0] + dir[0] * 100), int(center[1] + dir[1] * 100))
                    # print(center)

                    cv2.circle(img, center, 5, (0, 0, 255), -1)
                    cv2.arrowedLine(img, center, end, (0, 0, 255), 2)

        color_debug.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))
        if contours == []:
            flag = False
            break


kp_x = 0.01


def correct_pose(actual_arrows_center):
    set_velocity(vy=(actual_arrows_center[0] - 159) * -kp_x, vz=0, frame_id='body')


navigate_wait(frame_id='body', auto_arm=True)
# set_yaw(yaw=math.radians(0), frame_id='body')
threading.Thread(target=process_image, daemon=True).start()
rospy.sleep(2)

kp_yaw = 2.5

while flag:
    print('okok')

    actual_arrows_center = (px, py)
    # drone_pose = correct_pose(actual_arrows_center)
    print(actual_arrows_center)
    set_yaw_rate(yaw_rate=math.radians(angle) * -kp_yaw)
    correct_pose(actual_arrows_center)
    # if angle < 0:
    # if drone_pose == True:
    # else:
    # set_yaw_rate(yaw_rate=0.3)
    # navigate_wait(x = 0.2, y = 0, z = 0, frame_id='body')
    set_velocity(vx=0.25, vy=0, vz=0, frame_id='body')
    dist = rospy.wait_for_message('rangefinder/range', Range).range
    if dist < 0.2:
        navigate_wait(x=0, y=0, z=0.2, frame_id='body')

while not flag:
    print('bambam')
    aruco_pose()
