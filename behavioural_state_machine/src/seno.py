#! /usr/bin/env python

import numpy as np # to work with numerical data efficiently
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import JointState

amplitude = 200*np.pi/180
limitmax = amplitude/2
limitmin = -amplitude/2

position = 50*np.pi/180




fs = 500 # sample rate 
f = 3 # the frequency of the signal

x = np.arange(fs) # the points on the x axis for plotting
# compute the value (amplitude) of the sin wave at the for each sample
y = np.sin(2*np.pi*f * x / fs)*(amplitude/2) + position

for i in range(len(y)):
	if y[i] > limitmax:
		y[i] = limitmax
	elif y[i] < limitmin:
		y[i] = limitmin


plt.plot(x, y)
plt.plot(x, -y)
plt.xlabel('sample(n)')
plt.ylabel('angle')
plt.show()


#index on the msg joint_states
eyes_tilt_joint = 2
neck_pan_joint = 17
neck_tilt_joint = 18
version_joint = 32

#neck limits (degrees)
neck_pan_max = 53
neck_pan_min = -53
neck_tilt_max = 37
neck_tilt_min = -18

#eyes limits (degrees)
eye_max=38
eye_min=-38


#try:
#    cena = rospy.wait_for_message('/vizzy/joint_states', sensor_msgs.msg.JointState, timeout=10)
#    print cena
#except(rospy.ROSException), e:
#    print "Laser scan topic not available, aborting..."
#    print "Error message: ", e




def cb_once(msg):
	global var
	global i
	var = msg
	pub = rospy.Publisher('traj', JointState, queue_size=10)
	pub.publish(var)
	

	if i == 0:
		sub_once.unregister()
		rospy.signal_shutdown("shutting down")
	#rospy.signal_shutdown("shutting down")


	i=i+1

def listener():

	# In ROS, nodes are uniquely named. If two nodes with the same
	# name are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	#rospy.init_node('listener', anonymous=True)
	global sub_once
	sub_once = rospy.Subscriber('/vizzy/joint_states', JointState, cb_once)
	rospy.wait_for_message('/vizzy/joint_states', JointState)

	# spin() simply keeps python from exiting until this node is stopped
	#rospy.spin()


#def talker():





def main():
	rospy.init_node('listener', anonymous=True)
	listener()
#	talker()
	global i
	i = 0;
	rospy.spin()



if __name__ == '__main__':
	main()