#!/usr/bin/env python
# This is a listener for the OccupancyGrid publisher. Use as a
# sanity check if you mess with the other script.

import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/map', OccupancyGrid, mapCallback, queue_size=1)

    rospy.spin()

def mapCallback(data):
    print data.data

if __name__ == '__main__':
    listener()
