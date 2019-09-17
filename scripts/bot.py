#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import enum
import numpy


class BotState(enum.Enum):
    WAL_FOLLOW = 1
    GOAL_SEEK = 2  # type: BotState


currentBotState = BotState.GOAL_SEEK
maxRange = 0
minRange = 0
count = 1
PI = 3.1415926535897

# class SensorInfo:
#     def __init__(self, maxRange, minRange):
#         self.name = name
#         self.age = age
#
#     def myfunc(self):
#         print("Hello my name is " + self.name)


def callback(msg):
    print('=================================='+str(currentBotState.value))
    print('CHECKING .....')
    print(msg.pose.position)
    # goal_location.unregister()
    rospy.wait_for_message("homing_signal", PoseStamped)


def process_sensor_info(data):
    global maxRange, minRange
    maxRange = data.range_max
    minRange = data.range_min
    zone = numpy.array_split(numpy.array(data.ranges), 5)
    global zone_R, zone_FR, zone_F, zone_FL, zone_L
    zone_R = zone[0]
    zone_FR = zone[1]
    zone_F = zone[2]
    zone_FL = zone[3]
    zone_L = zone[4]
    # global count
    # if count == 1:
    #     bot_bug2()
    #     count = count + 1


def bot_bug2():
    bot_motion = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10)
    # Converting from angles to radians
    speed = 90
    angle = 90
    angular_speed = speed * 2 * PI / 360
    relative_angle = angle * 2 * PI / 360
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = abs(angular_speed)
    t0 = rospy.Time.now().to_sec()
    while t0 == 0:
        t0 = rospy.Time.now().to_sec()
    # print("--------->")
    # print(t0)
    # print("--------->")
    current_angle = 0
    while (current_angle < relative_angle):
        # current_angle = current_angle + 1
        # print(current_angle)
        bot_motion.publish(twist)
        t1 = rospy.Time.now().to_sec()
        #print(t1)
        current_angle = angular_speed * (t1-t0)
        # print(relative_angle)
        rate.sleep()
    print(t0)
    twist.angular.z = 0
    bot_motion.publish(twist)


def get_home_location():
    global goal_location
    rospy.init_node("getHomeLocation", anonymous=True)
    goal_location = rospy.Subscriber('/homing_signal', PoseStamped, callback)
    read_sensor_data()
    bot_bug2()
    rospy.spin()


def read_sensor_data():
    rospy.Subscriber('/base_scan', LaserScan, process_sensor_info)


if __name__ == '__main__':
    try:
        get_home_location()
    except rospy.ROSInterruptException:
        pass
