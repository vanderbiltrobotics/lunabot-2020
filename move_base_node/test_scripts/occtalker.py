#!/usr/bin/env python
# The following is a publisher which creates a dummy OccupancyGrid
# and publishes it.

import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np

def talker():
    pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        grid_msg = OccupancyGrid()
	# Header
	grid_msg.header.stamp = rospy.Time.now()
	grid_msg.header.frame_id = "map"
	# MapMetaData
	grid_msg.info.resolution = 100
	grid_msg.info.width = 100
	grid_msg.info.height = 100
	grid_msg.info.origin = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
	# Data
	grid_msg.data = [100, 100, 0, 100, 0, 100, 0, 100]
	# Uncomment this if you want to see an endless loop of
	# the published data.
        #rospy.loginfo(grid_msg)
        pub.publish(grid_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
