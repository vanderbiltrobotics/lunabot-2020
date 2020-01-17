#!usr/bin/python

import rospy
import smach

## State Declarations ##


# QUICK REFERENCE: 
# 
# During the init method, perform any setup work. DO NOT BLOCK in the init method.
# Write all state code in execute method. All blocking should be done here. 
#
#

class LocalizingWithCharuco(smach.State):
	def __init__(self, outcomes=['Localized', 'Lost']):
		# TODO: Start localization here (async)

	def execute(self, data):
		try:
			LocalizeWithCharuco() # TODO: Implement, @Charuco Team?
		except:
			return 'Lost'
			
		return 'Localized'


class DriveToDigSpot(smach.State):
	def __init__(self, outcomes=['AtDigSpot', 'Lost']):
	
	def execute(self, data):
		try:
			DriveToDigSpot()
		except:
			return 'Lost'
			
		return 'AtDigSpot'


class Excavate(smach.State):
	def __init__(self, outcomes=['Excavated']):

	def execute(self, data):
		Excavate() # TODO: Implement

		return 'Excavated'


class LocalizingNoCharuco(smach.State):
	def __init__(self, outcomes=['DriveToBin']):

	def execute(self, data):
		LocalizeNoCharuco() # TODO: Implement, @mapping team

		return 'DriveToBin'


class DriveToBin(smach.State):
	def __init__(self, outcomes=['AtBin', 'Lost']):

	def execute(self, data):
		try:
			DriveToBin() # TODO: Implement
		except:
			return 'Lost'

		return 'AtBin'


class Deposit(smach.State):
	def __init__(self, outcomes=['Deposited']):

	def execute(self, data):
		while Depositing():
			pass
		return 'Deposited'


