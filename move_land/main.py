import math
import rospy
from clover import srv
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandBool, CommandLong
from pymavlink import mavutil


rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

send_command = rospy.ServiceProxy('mavros/cmd/command', CommandLong)

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)


navigate_wait(z=1.5, frame_id='body', auto_arm=True)
navigate_wait(x=2, y=2, z=1.5, frame_id='aruco_map')

rospy.sleep(2)
telem = get_telemetry(frame_id='aruco_141')
print(telem.x, telem.y)

# while True:
#     distance = (telem.x ** 2 + telem.y ** 2) ** 0.5
#     height_aruco = get_telemetry(frame_id='aruco_map').z
#     while height_aruco > 0.35:
#         height_aruco = get_telemetry(frame_id='aruco_map').z
#         telem = get_telemetry(frame_id='aruco_141')
#         distance = (telem.x ** 2 + telem.y ** 2) ** 0.5
#         h = distance * 2 + 0.2
#         if h > 1.5:
#             h = 1.5
#         if h is None and height_aruco >= 0.6:
#             navigate(x=0, y=0, z=1.5, frame_id='body')
#         elif height_aruco < 0.6:
#             break
#         else:
#             navigate(x=0, y=0, z=h, yaw=float('nan'), frame_id='aruco_141')
#         print(distance, h, height_aruco)
#
#
#     if get_telemetry(frame_id='aruco_map').z < 0.45:
#         send_command(command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, param1=0, param2=21196)
#         print('land successful!')
#         break
#     else:
#         print('unlucky, trying again!')

while True:
    while True:
        height_aruco = get_telemetry(frame_id='aruco_map').z
        telem = get_telemetry(frame_id='aruco_141')
        distance = (telem.x ** 2 + telem.y ** 2) ** 0.5

        h = distance * 2 + 0.2

        if not type(distance) is float or (height_aruco < 0.45 and distance < 0.15):
            print('break the while')
            break
        else:
            navigate(x=0, y=0, z=h, yaw=float('nan'), frame_id='aruco_141')

        print(height_aruco, distance)

    #if get_telemetry(frame_id='aruco_map').z < 0.45:
    send_command(command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, param1=0, param2=21196)
    print('land successful')
    break
    #else:
    #    navigate_wait(x=2, y=2, z=1.5, frame_id='aruco_map')
    #print('trying again')

print('i resign')
