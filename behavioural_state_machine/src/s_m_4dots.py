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
    else:
        return 'base_reset'

def monitor_cb1(ud, msg):
    if msg.data == 'Phone_call':
        return False
    else:
        return True

def monitor_cb2(ud, msg):
    if msg.data == 'Pay_attention':
        return False
    else:
        return True

def monitor_cb3(ud, msg):
    if msg.data == 'Have_a_question':
        return False
    else:
        return True

def monitor_cb4(ud, msg):
    if msg.data == 'Go_out':
        return False
    else:
        return True


# main
def main():

    rospy.init_node('s_m', anonymous=True)


    base = smach.Concurrence(outcomes=['looking1', 'looking2', 'looking3', 'looking4','base_reset'],
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



    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done'])

    # Open the container
    with sm:


        smach.StateMachine.add('SETUP', Setup(), transitions={'setup_done':'BASE'})
        smach.StateMachine.add('BASE', base, transitions={'looking1':'GOAL_1',
        													'looking2':'GOAL_2',
        													'looking3':'GOAL_3',
        													'looking4':'GOAL_4',
                                                            'base_reset':'BASE'})





        def goal_callback1(userdata, default_goal):
			goal = vizzy_msgs.msg.GazeGoal()
			goal.type = vizzy_msgs.msg.GazeGoal.CARTESIAN
			goal.fixation_point_error_tolerance = 0.01
			goal.fixation_point.point.x = 2
			goal.fixation_point.point.y = 0
			goal.fixation_point.point.z = 0
			goal.fixation_point.header.frame_id='base_footprint'
			goal.fixation_point.header.stamp=rospy.get_rostime()
			return goal

        smach.StateMachine.add('GOAL_1',
                                smach_ros.SimpleActionState('gaze', vizzy_msgs.msg.GazeAction,
                                                        goal_cb = goal_callback1),
                                {'succeeded':'BASE',
                                 'preempted':'done',
                                 'aborted':'done'})





        def goal_callback2(userdata, default_goal):
			goal = vizzy_msgs.msg.GazeGoal()
			goal.type = vizzy_msgs.msg.GazeGoal.CARTESIAN
			goal.fixation_point_error_tolerance = 0.01
			goal.fixation_point.point.x = 2
			goal.fixation_point.point.y = 0
			goal.fixation_point.point.z = 2
			goal.fixation_point.header.frame_id='base_footprint'
			goal.fixation_point.header.stamp=rospy.get_rostime()
			return goal

        smach.StateMachine.add('GOAL_2',
                                smach_ros.SimpleActionState('gaze', vizzy_msgs.msg.GazeAction,
                                                        goal_cb = goal_callback2),
                                {'succeeded':'BASE',
                                'preempted':'done',
                                 'aborted':'done'})






        def goal_callback3(userdata, default_goal):
			goal = vizzy_msgs.msg.GazeGoal()
			goal.type = vizzy_msgs.msg.GazeGoal.CARTESIAN
			goal.fixation_point_error_tolerance = 0.01
			goal.fixation_point.point.x = 2
			goal.fixation_point.point.y = 1
			goal.fixation_point.point.z = 1
			goal.fixation_point.header.frame_id='base_footprint'
			goal.fixation_point.header.stamp=rospy.get_rostime()
			return goal

        smach.StateMachine.add('GOAL_3',
                                smach_ros.SimpleActionState('gaze', vizzy_msgs.msg.GazeAction,
                                                        goal_cb = goal_callback3),
                                {'succeeded':'BASE',
                                'preempted':'done',
                                 'aborted':'done'})






        def goal_callback4(userdata, default_goal):
			goal = vizzy_msgs.msg.GazeGoal()
			goal.type = vizzy_msgs.msg.GazeGoal.CARTESIAN
			goal.fixation_point_error_tolerance = 0.01
			goal.fixation_point.point.x = 2
			goal.fixation_point.point.y = -1
			goal.fixation_point.point.z = 1
			goal.fixation_point.header.frame_id='base_footprint'
			goal.fixation_point.header.stamp=rospy.get_rostime()
			return goal

        smach.StateMachine.add('GOAL_4',
                                smach_ros.SimpleActionState('gaze', vizzy_msgs.msg.GazeAction,
                                                        goal_cb = goal_callback4),
                                {'succeeded':'BASE',
                                'preempted':'done',
                                 'aborted':'done'})



	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()
    # Execute SMACH plan
    outcome = sm.execute()

	#sis.stop()

if __name__ == '__main__':
    main()
