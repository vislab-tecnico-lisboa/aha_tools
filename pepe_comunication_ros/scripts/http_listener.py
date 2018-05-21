#!/usr/bin/env python
from flask import Flask, url_for, request
import rospy
import rospkg
from std_msgs.msg import String




app = Flask(__name__)
publisher = rospy.Publisher("/test_http",String,queue_size=10)

@app.route('/',methods=['POST'])
def publish_to_topic():
    publisher.publish(request.data)
    return "Sucess"

rospy.init_node('http2ros',log_level=rospy.DEBUG)
app.run()
