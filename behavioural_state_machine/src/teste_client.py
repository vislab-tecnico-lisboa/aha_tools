#! /usr/bin/env python

import roslib; roslib.load_manifest('behavioural_state_machine')
import rospy
import actionlib
import behavioural_state_machine.msg

def teste_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('teste', behavioural_state_machine.msg.TesteAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = behavioural_state_machine.msg.TesteGoal(movement = 'y')

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('teste_client_py')
        result = teste_client()
        print ("Succeded")
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
