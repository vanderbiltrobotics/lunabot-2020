#!/usr/bin/env python

# ----------------------------------------------- #
# Table of index number of /joy.buttons:
# Index   Button name on the actual controller
#
# 0   A (Lead Screw)
# 1   B (Buckets)
# 2   X (Hinge)
# 3   Y (Conveyor)
# 4   LB
# 5   RB
# 6   back (accel_decel)
# 7   start (direction)
# 8   power
# 9   Button stick left
# 10  Button stick right

# Table of index number of /joy.axes:
# Index   Axis name on the actual controller
#
# 0   Moving left joystick left (+) and right (-) changes rotational velocity
# 1   Moving left joystick up (+) and down (-) changes linear velocity
# 2   LT
# 3   Left/Right Axis stick right
# 4   Up/Down Axis stick right
# 5   RT
# 6   cross key left/right      (buckets / hinge accel)
# 7   cross key up/down         (hinge / lead screw accel)
# ----------------------------------------------- #

# import required packages
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, String


class TeleopControl:

    # Constructor
    def __init__(self):
        # Initialize drive speed publishers
        self.publishers = {
            "pub_drive_cmd": rospy.Publisher('drive_cmd', Twist, queue_size=0)
        }

        # Initialize subscribers
        self.joy_sub = rospy.Subscriber("joy", Joy, self.process_joystick_data)

    # --- CALLBACK FUNCTIONS --- #

    # Callback for joystick data
    def process_joystick_data(self, msg):
        # Get motor velocity commands
        lin_vel = msg.axes[1]
        ang_vel = msg.axes[3]
        """
                # Get direction and accel_decel toggle states
                acc_dir_toggle = msg.buttons[5]

                # Update accel_direction
                if self.state["acc_dir_toggle"] == 1 and acc_dir_toggle == 0:
                    self.state["acc_dir"] *= -1.0

                # Update dir and acc toggle
                self.state["acc_dir_toggle"] = acc_dir_toggle

        """
        # --- PUBLISH COMMAND MESSAGES --- #

        # Drive twist
        drive_msg=Twist()
        drive_msg.linear.x=lin_vel
        drive_msg.angular.z=ang_vel
        self.publishers["pub_drive_cmd"].publish(drive_msg)

if __name__ == '__main__':
    # Initialize as ROS node
    rospy.init_node('teleop_control')

    # Create a TeleopControl object
    control = TeleopControl()

    # Ready to go
    rospy.loginfo("Teleop Control initialized...")

    # Loop continuously
    rospy.spin()
