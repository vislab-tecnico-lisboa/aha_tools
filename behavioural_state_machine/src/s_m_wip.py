#!/usr/bin/env python

import roslib;
import rospy
import smach
import smach_ros
from std_msgs.msg import String

import actionlib
import vizzy_msgs.msg






class Setup(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['setup_done'])
	def execute(self, userdata):
		return 'setup_done'






class Waiting(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted'])
	def execute(self, userdata):
		while True:
			if self.preempt_requested():
				self.service_preempt()
				return 'preempted'

def child_term_cb(outcome_map):
	if outcome_map['SENSOR1'] == 'invalid':
		return True
	elif outcome_map['SENSOR2'] == 'invalid':
		return True
	elif outcome_map['SENSOR3'] == 'invalid':
		return True
	elif outcome_map['SENSOR4'] == 'invalid':
		return True
	elif outcome_map['SENSOR5'] == 'invalid':
		return True
	else:
		return False

def out_cb(outcome_map):
	if outcome_map['SENSOR1'] == 'invalid':
		return 'looking1'
	elif outcome_map['SENSOR2'] == 'invalid':
		return 'looking2'
	elif outcome_map['SENSOR3'] == 'invalid':
		return 'looking3'
	elif outcome_map['SENSOR4'] == 'invalid':
		return 'looking4'
	elif outcome_map['SENSOR5'] == 'invalid':
		return 'sim'
	else:
		return 'base_reset'

def monitor_cb1(ud, msg):
	if msg.data == 'Phone_call':
		return False
	else:
		return True

def monitor_cb2(ud, msg):
	if msg.data == 'Go_out':
		return False
	else:
		return True

def monitor_cb3(ud, msg):
	if msg.data == 'Pay_attention':
		return False
	else:
		return True

def monitor_cb4(ud, msg):
	if msg.data == 'Have_a_question':
		return False
	else:
		return True

def monitor_cb5(ud, msg):
	if msg.data == 'sim':
		return False
	else:
		return True




def gazeclient(x,y,z,w,tol):
	# Creates the SimpleActionClient, passing the type of the action
	# to the constructor.
	client = actionlib.SimpleActionClient('gaze', vizzy_msgs.msg.GazeAction)

	# Waits until the action server has started up and started
	# listening for goals.
	client.wait_for_server()

	goal = vizzy_msgs.msg.GazeGoal()
	goal.type = vizzy_msgs.msg.GazeGoal.CARTESIAN
	if tol != 0:
		goal.fixation_point_error_tolerance = tol
	else:
		goal.fixation_point_error_tolerance = 0.01
	goal.fixation_point.point.x = x
	goal.fixation_point.point.y = y
	goal.fixation_point.point.z = z
	goal.fixation_point.header.frame_id='base_footprint'
	goal.fixation_point.header.stamp=rospy.get_rostime()

	# Sends the goal to the action server.
	client.send_goal(goal)

	# Waits for the server to finish performing the action.
	if w == 1:
		client.wait_for_result()

	# Prints out the result of executing the action
	return client.get_result()



class gaze1(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded'])
	def execute(self, userdata):

		client = gazeclient(2,0,0,0,0)
		return 'succeeded'

class gaze2(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded'])
	def execute(self, userdata):

		client = gazeclient(2,0,2,0,0)
		return 'succeeded'

class gaze3(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded'])
	def execute(self, userdata):

		client = gazeclient(2,1,1,0,0)
		return 'succeeded'

class gaze4(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded'])
	def execute(self, userdata):

		client = gazeclient(2,-1,1,0,0)
		return 'succeeded'



class afirmativo(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded'])
	def execute(self, userdata):
		x=0

		while x < 4:
			cima = gazeclient(2,0,1.6,1,0.7)
			baixo = gazeclient(2,0,0,1,0.8)
			x = x+1
		client3 = gazeclient(2,0,1,0,0)
		return 'succeeded'




# main
def main():

	rospy.init_node('s_m', anonymous=True)


	base = smach.Concurrence(outcomes=['looking1', 'looking2', 'looking3', 'looking4',
										'sim','base_reset'],
											default_outcome='base_reset',
											child_termination_cb=child_term_cb,
											outcome_cb=out_cb)

	with base:
		smach.Concurrence.add('WAITING', Waiting())
		smach.Concurrence.add('SENSOR1', smach_ros.MonitorState("chat",
																String, monitor_cb1))
		smach.Concurrence.add('SENSOR2', smach_ros.MonitorState("chat",
																String, monitor_cb2))
		smach.Concurrence.add('SENSOR3', smach_ros.MonitorState("chat",
																String, monitor_cb3))
		smach.Concurrence.add('SENSOR4', smach_ros.MonitorState("chat",
																String, monitor_cb4))
		smach.Concurrence.add('SENSOR5', smach_ros.MonitorState("chat",
																String, monitor_cb5))


	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['done'])

	# Open the container
	with sm:


		smach.StateMachine.add('SETUP', Setup(), transitions={'setup_done':'BASE'})
		smach.StateMachine.add('BASE', base, transitions={'looking1':'GOAL_1',
															'looking2':'GOAL_2',
															'looking3':'GOAL_3',
															'looking4':'GOAL_4',
															'sim':'SIM',
															'base_reset':'BASE'})



		smach.StateMachine.add('GOAL_1', gaze1(), {'succeeded':'BASE'})

		smach.StateMachine.add('GOAL_2', gaze2(), {'succeeded':'BASE'})

		smach.StateMachine.add('GOAL_3', gaze3(), {'succeeded':'BASE'})

		smach.StateMachine.add('GOAL_4', gaze4(), {'succeeded':'BASE'})

		smach.StateMachine.add('SIM', afirmativo(), {'succeeded':'BASE'})



	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()
	# Execute SMACH plan
	outcome = sm.execute()

	#sis.stop()

if __name__ == '__main__':
	main()