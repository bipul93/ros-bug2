#!/usr/bin/env python
import math

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import enum
import numpy


class BotState(enum.Enum):
    LOOK_TOWARDS = 0
    GOAL_SEEK = 1  # type: BotState
    WALL_FOLLOW = 2


yaw = 0
yaw_threshold = math.pi / 90 # +/- 2 degree allowed
currentBotState = BotState.LOOK_TOWARDS
maxRange = 3
minRange = 0

bot_pose = None
init_bot_pose = []
beacon_pose = None
bot_motion = None
homing_signal = None
init_config_complete = False
wall_hit_point = None
beacon_found = False
twist = Twist()
distance_moved = 0

wall_follow_mode = False
line_point_found = False


def normalize(angle):
    if math.fabs(angle) > math.pi:
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def look_towards(des_pos):
    global yaw, yaw_threshold, bot_motion, currentBotState, twist
    quaternion = (
        des_pos.orientation.x,
        des_pos.orientation.y,
        des_pos.orientation.z,
        des_pos.orientation.w)
    euler = euler_from_quaternion(quaternion)
    yaw = euler[2]
    beacon_yaw = math.atan2(beacon_pose.position.y - des_pos.position.y, beacon_pose.position.x - des_pos.position.x)
    yaw_diff = normalize(beacon_yaw - yaw)

    if math.fabs(yaw_diff) > yaw_threshold:
        twist.angular.z = -0.5  # clockwise rotation if yaw_diff > 0 else 0.5  # counter-clockwise rotation

    bot_motion.publish(twist)

    if math.fabs(yaw_diff) <= yaw_threshold:
        twist.angular.z = 0
        currentBotState = BotState.GOAL_SEEK


def goal_seek():
    global zone_F, currentBotState, bot_pose, wall_hit_point
    # zone_F = numpy.array(zone_F)
    obstacle_in_front = numpy.any((zone_F < 0.75))
    # Or find the minimum value in this zone. or maybe numpy.any would be faster
    # print(obstacle_in_front)
    if obstacle_in_front:
        twist.linear.x = 0
        wall_hit_point = bot_pose.position
        currentBotState = BotState.WALL_FOLLOW
    else:
        twist.angular.z = 0
        twist.linear.x = 0.5
    bot_motion.publish(twist)


def wall_follow():
    global wall_follow_mode, line_point_found, twist, bot_pose, bot_motion, currentBotState, distance_moved, wall_hit_point
    wall_follow_mode = True
    line_point_found = False

    # Todo: Tune the parameters.
    # maybe turn right until zone_F is clear
    # Wall follow enter
    # Optimize this piece of logic
    obstacle_in_front = numpy.any((zone_F < 0.75))
    distance_moved = math.sqrt(pow(bot_pose.position.y - wall_hit_point.y, 2) + pow(bot_pose.position.x - wall_hit_point.x, 2))
    # print(line_distance(), distance_moved, (line_distance() < 0.2 and distance_moved > 0.5))
    if line_distance() < 0.2 and distance_moved > 0.5:
        print("line_hit")
        print(distance_moved)
        # found line point. rotate and move forward
        twist.angular.z = 0
        twist.linear.x = 0
        currentBotState = BotState.LOOK_TOWARDS
    elif obstacle_in_front:  # turn right
        twist.angular.z = -0.5
        twist.linear.x = 0
    elif numpy.all((zone_FL >= 2)):  # turn left maybe TODO: what if there's wall on left and right both sides.
        twist.angular.z = 0.5
        twist.linear.x = 0
    else:
        twist.angular.z = 0  # move forward
        twist.linear.x = 0.5

    bot_motion.publish(twist)


def line_distance():
    global init_bot_pose, beacon_pose, bot_pose
    point_1 = init_bot_pose  # in form of array
    point_2 = beacon_pose.position
    point_k = bot_pose.position
    numerator = math.fabs((point_2.y - point_1[1]) * point_k.x - (point_2.x - point_1[0]) * point_k.y + (point_2.x * point_1[1]) - (point_2.y * point_1[0]))
    denominator = math.sqrt(pow(point_2.y - point_1[1], 2) + pow(point_2.x - point_1[0], 2))
    return numerator / denominator


def callback(msg):
    print('CHECKING .....')
    global beacon_pose
    beacon_pose = msg.pose
    check_init_config()
    # goal_location.unregister()
    rospy.wait_for_message("homing_signal", PoseStamped)


def get_base_truth(bot_data):
    global bot_pose, beacon_found
    bot_pose = bot_data.pose.pose
    if not init_config_complete:
        check_init_config()
    goal_distance = math.sqrt(pow(bot_pose.position.y - beacon_pose.position.y, 2) + pow(bot_pose.position.x - beacon_pose.position.x, 2))
    if goal_distance <= 0.2:
        beacon_found = True


# ToDo: merge these two functions with type check conditions get_base_truth and Process_sensor_info


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


def check_init_config():
    global bot_pose, beacon_pose, init_config_complete, init_bot_pose
    if bot_pose is not None and beacon_pose is not None:
        init_config_complete = True
        init_bot_pose = [bot_pose.position.x, bot_pose.position.y]
        bot_bug2()


def bot_bug2():
    global bot_motion, currentBotState, bot_pose
    bot_motion = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(10)
    while not beacon_found:
        if not init_config_complete:
            return
        if currentBotState is BotState.LOOK_TOWARDS:
            # print("look towards")
            look_towards(bot_pose)
        elif currentBotState is BotState.GOAL_SEEK:
            # print("Goal Seek")
            goal_seek()
        elif currentBotState is BotState.WALL_FOLLOW:
            # print("Wall Follow")
            wall_follow()
            # return

        # print("Beacon not found yet")
        rate.sleep()


def init():
    global homing_signal
    rospy.init_node("bot", anonymous=True)
    homing_signal = rospy.Subscriber('/homing_signal', PoseStamped, callback)
    rospy.Subscriber('/base_scan', LaserScan, process_sensor_info)
    rospy.Subscriber('/base_pose_ground_truth', Odometry, get_base_truth)

    rospy.spin()


if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
