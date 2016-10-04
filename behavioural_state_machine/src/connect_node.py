#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from pymongo import MongoClient
from pymongo import CursorType
import time


def talker():


	#client = MongoClient()
	#client = MongoClient('localhost', 27017)
	#client = MongoClient('mongodb://10.0.27.117:27017/')
	client = MongoClient('mongodb://192.168.1.153:27017/')


	db = client.AHA
	coll = db.GestureBuildEvents
	cursor = coll.find(cursor_type = CursorType.TAILABLE_AWAIT)

	pub = rospy.Publisher('chat', String, queue_size=100)

	rospy.init_node('talker', anonymous=True)

	while cursor.alive:
	    try:
	        doc = cursor.next()
	        if doc["confidence"] > 0.1:
	        	rospy.loginfo(doc["gesture"])
	        	pub.publish(doc["gesture"])
	    except StopIteration:
	        time.sleep(1)



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
