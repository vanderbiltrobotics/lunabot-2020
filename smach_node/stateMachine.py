import rospy
import smach


# Import States
import smachStates


StateMachine = smach.StateMachine():

with StateMachine:
	smach.StateMachine.add('LOCALIZING_WITH_CHARUCO', LocalizeWithCharuco(),
		transitions={'Localized':'DRIVE_TO_DIG_SPOT',
				'Lost':'LOCALIZING_NO_CHARUCO'})

	smach.StateMachine.add('DRIVING_TO_DIG_SPOT', DrivingToDigSpot(),
		transitions={'AtDigSpot':'EXCAVATING',
				''}

	smach.StateMachine.add('EXCAVATING', Excavating(),
		transitions={'Excavated':'LOCALIZING_NO_CHARUCO'})

# TODO: Finish boilerplate
