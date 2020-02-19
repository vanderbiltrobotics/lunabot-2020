#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Float32, Header
from sensor_msgs.msg import JointState
import math

"""Init constants with regards to the physical model of the arm"""
LINK_1_LENGTH = .413
LINK_2_LENGTH = .365
JOINT_1_LIM = [0, math.pi * 3 / 4]  # These are arbitrary and likely will have to change.
JOINT_2_LIM = [0, math.pi * 3 / 4]  # Angles are defined as 0 when all joints are pointing straight up
JOINT_3_LIM = [-math.pi *3/4, math.pi * 3 / 4]
#shitty implementation

"""What this node should do:"""
"""Receives Pose2D msg's, transforms them into joint positions, and then into linear actuator positions. """
"""Also receives current msg's from the Talons and can give a stop command if necessary"""


class ExcavationCommand:

    def cmd_callback(self,msg):
        ang = self.ik(msg)
        if self.current_bad:
            rospy.loginfo("Current Spike! Stopping Movements...")
            msg2 = rospy.wait_for_message('pot',Float32MultiArray)
            self.pot_pub.publish(msg2)
            msg3 = self.pot_to_angle(msg2)
            self.ang_pub.publish(msg3)
        elif ang == -1:
            rospy.loginfo("Invalid pose!")
        else:
            rospy.loginfo("Good pose! Publishing..."+str(ang.data[0])+" "+str(ang.data[1])+" "+str(ang.data[2]))
            self.ang_pub.publish(ang)
            pot = self.angle_to_pot(ang)
            self.pot_pub.publish(ang)
            h = Header()
            h.stamp = rospy.Time.now()
            joint_state = JointState()
            joint_state.header= h
            joint_state.name = ["base_to_arm_1", "arm_1_to_arm_2", "arm_2_to_arm_3", "left_front_wheel_joint", "left_back_wheel_joint",
  "right_front_wheel_joint", "right_back_wheel_joint"]
            joint_state.position = ang.data+[0, 0, 0, 0]
            self.joint_pub.publish(joint_state)



    def talon_callback(self,msg):
        self.current_bad = self.check_current(msg)

    def __init__(self):
        while not rospy.is_shutdown():
            rospy.init_node('excavation_cmd')
            self.current_bad = False

            # Initialize subscribers
            self.cmd_sub = rospy.Subscriber("arm_cmd", Pose2D, self.cmd_callback)
            self.talon_sub = rospy.Subscriber("current", Float32, self.talon_callback)

            # Publishers, ang_pub is for simulation purposes
            self.pot_pub = rospy.Publisher('pot_arm_cmd', Float32MultiArray, queue_size=5)
            self.ang_pub = rospy.Publisher('ang_arm_cmd', Float32MultiArray, queue_size=5) #dunno if we need this
            self.joint_pub = rospy.Publisher('joint_states',JointState, queue_size=5) #for rviz
            rospy.loginfo("excavation_cmd init'd")
            rospy.spin() #and we wait...

    """returns list of joint angles to achieve pose. Angles are defined as 0 when all joints are pointing straight up."""
    """Positive angles are clockwise from vertical, when the center of the robot is to the right."""
    """So far does no checks as to whether or not the pose_2d should not be attempted (hitting the ground, or the bucket, etc)"""
    def ik(self, pose_2d):
        x = pose_2d.x
        y = pose_2d.y
        theta = pose_2d.theta  # please send me values from -pi to pi
        try:
            """Math is here, always takes the "elbow up" solution"""
            k = x * x + y * y - math.pow(LINK_1_LENGTH, 2) - math.pow(LINK_2_LENGTH, 2)
            theta2 = math.atan2(math.sqrt(1 - math.pow(k / 2 / LINK_1_LENGTH / LINK_2_LENGTH, 2)),
                                k / 2 / LINK_1_LENGTH / LINK_2_LENGTH)
            if theta2 < 0:
                theta2 = -theta2
            theta1 = math.atan2(x, y) - math.atan2(LINK_2_LENGTH * math.sin(theta2),
                                                   LINK_1_LENGTH + LINK_2_LENGTH * math.cos(theta2))
            theta3 = theta - theta1 - theta2
        except ValueError:
            return -1
        if (theta1 > JOINT_1_LIM[0]) & (theta1 < JOINT_1_LIM[1]) & (theta2 > JOINT_2_LIM[0]) & (
                theta2 < JOINT_2_LIM[1]) & (theta3 > JOINT_3_LIM[0]) & (theta3 < JOINT_3_LIM[1]):
            ans = Float32MultiArray()
            ans.data = [theta1, theta2, theta3]
        else:
            ans = -1 # Need to think a better failure condition

        return ans

    """Converts potentiometer values to angles"""
    def pot_to_angle(self, pot):
        return pot  # We don't know the func yet

    """Converts joint angles to potentiometer values"""
    def angle_to_pot(self, ang):
        return ang  # Same deal

    """This is currently a really dumb method, more of a reminder to implement this later when we know more about what arm failing looks like"""
    def check_current(self,current):
        if current >1: return True
        else: return False




if __name__ == '__main__':
    foo = ExcavationCommand()
