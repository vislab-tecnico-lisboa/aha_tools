#!/usr/bin/env python

import roslib;
import rospy
import smach
import smach_ros
from std_msgs.msg import String

import actionlib
import vizzy_msgs.msg
import behavioural_state_machine.msg






class Setup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['setup_done'])
    def execute(self, userdata):
        rospy.sleep(1)
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
    elif outcome_map['SENSOR_YES'] == 'invalid':
        return True
    elif outcome_map['SENSOR_NO'] == 'invalid':
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
    elif outcome_map['SENSOR_YES'] == 'invalid':
        return 'moving_yes'
    elif outcome_map['SENSOR_NO'] == 'invalid':
        return 'moving_no'
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

def monitor_cb_yes(ud, msg):
    if msg.data == 'Confident_1':
        return False
    else:
        return True

def monitor_cb_no(ud, msg):
    if msg.data == 'Stop':
        return False
    else:
        return True






def gazeclient(x,y,z):
	# Creates the SimpleActionClient, passing the type of the action
	# (vizzy_msgs.msg.GazeAction) to the constructor.
	client = actionlib.SimpleActionClient('gaze', vizzy_msgs.msg.GazeAction)

	# Waits until the action server has started up and started
	# listening for goals.
	client.wait_for_server()

	goal = vizzy_msgs.msg.GazeGoal()
	goal.type = vizzy_msgs.msg.GazeGoal.CARTESIAN
	goal.fixation_point_error_tolerance = 0.01
	goal.fixation_point.point.x = x
	goal.fixation_point.point.y = y
	goal.fixation_point.point.z = z
	goal.fixation_point.header.frame_id='base_footprint'
	goal.fixation_point.header.stamp=rospy.get_rostime()

	# Sends the goal to the action server.
	client.send_goal(goal)

	# Waits for the server to finish performing the action.
	#client.wait_for_result()

	return client.get_result()






def yesnoclient(mov):
    # Creates the SimpleActionClient, passing the type of the action
    # (behavioural_state_machine.msg.TesteAction) to the constructor.
    client = actionlib.SimpleActionClient('yesno', behavioural_state_machine.msg.TesteAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = behavioural_state_machine.msg.TesteGoal(movement = mov)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult








class gaze1(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded'])
	def execute(self, userdata):

		client = gazeclient(2,0,0)
		return 'succeeded'

class gaze2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
    def execute(self, userdata):

        client = gazeclient(2,0,2)
        return 'succeeded'

class gaze3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
    def execute(self, userdata):

        client = gazeclient(2,1,1)
        return 'succeeded'

class gaze4(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
    def execute(self, userdata):

        client = gazeclient(2,-1,1)
        return 'succeeded'

class yes(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
    def execute(self, userdata):

        client = yesnoclient('y')
        return 'succeeded'

class no(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
    def execute(self, userdata):

        client = yesnoclient('n')
        return 'succeeded'




# main
def main():

    rospy.init_node('s_m', anonymous=True)


    base = smach.Concurrence(outcomes=['looking1', 'looking2', 'looking3', 'looking4',
                                        'moving_yes', 'moving_no','base_reset'],
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
        smach.Concurrence.add('SENSOR_YES', smach_ros.MonitorState("chat",
                                                                String, monitor_cb_yes))
        smach.Concurrence.add('SENSOR_NO', smach_ros.MonitorState("chat",
                                                                String, monitor_cb_no))

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done'])

    # Open the container
    with sm:


        smach.StateMachine.add('SETUP', Setup(), transitions={'setup_done':'BASE'})
        smach.StateMachine.add('BASE', base, transitions={'looking1':'GOAL_1',
        													'looking2':'GOAL_2',
        													'looking3':'GOAL_3',
        													'looking4':'GOAL_4',
                                                            'moving_yes':'GOAL_YES',
                                                            'moving_no':'GOAL_NO',
                                                            'base_reset':'BASE'})



        smach.StateMachine.add('GOAL_1', gaze1(), {'succeeded':'BASE'})

        smach.StateMachine.add('GOAL_2', gaze2(), {'succeeded':'BASE'})

        smach.StateMachine.add('GOAL_3', gaze3(), {'succeeded':'BASE'})

        smach.StateMachine.add('GOAL_4', gaze4(), {'succeeded':'BASE'})

        smach.StateMachine.add('GOAL_YES', yes(), {'succeeded':'BASE'})

        smach.StateMachine.add('GOAL_NO', no(), {'succeeded':'BASE'})


	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()
    # Execute SMACH plan
    outcome = sm.execute()

	#sis.stop()

if __name__ == '__main__':
    main()