import rospy
import math
from sensor_msgs.msg import LaserScan

step = 3
num = 10000.0
res = 0.1
dist = 0.1 * res
pub = rospy.Publisher('/base_scan', LaserScan, queue_size=10)
pub_new = rospy.Publisher('/fixed_scan', LaserScan, queue_size=10)


def process_msg(msg, old=pub, new=pub_new):
    old.publish(msg)
    msg = laser_fixing(msg, msg.angle_min, msg.angle_increment)
    new.publish(msg)


def distance(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


def get_point(range, theta):
    return range * math.cos(theta), range * math.sin(theta)


def get_angle(angle, angle_increment, position):
    return angle + angle_increment * position


def laser_fixing(msg, angle, increment):
    data = []
    ranges = msg.ranges

    for i in range(step, len(ranges) - step):
        prev_step_point = get_point(ranges[i - step], get_angle(angle, increment, i - step))
        current_point = get_point(ranges[i], get_angle(angle, increment, i))
        next_step_point = get_point(ranges[i + step], get_angle(angle, increment, i + step))

        if distance(prev_step_point, current_point) > dist or distance(next_step_point, current_point) > dist:
            data.append(num)
        else:
            data.append(data[i])

    msg.ranges = [num] * step + data + [num] * step
    return msg


rospy.init_node('lr1')
rospy.Subscriber('base_scan', LaserScan, process_msg)
r = rospy.Rate(0.5)

while not (rospy.is_shutdown()):
    r.sleep()
