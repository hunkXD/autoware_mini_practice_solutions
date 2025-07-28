#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

rospy.init_node('publisher')
rate_hz = rospy.get_param("~rate", 1)  # Default to 1Hz if not set
rate = rospy.Rate(rate_hz)
# rate = rospy.Rate(2)

pub = rospy.Publisher('/message', String, queue_size=10)
message = rospy.get_param('~message', 'Hello World!')

while not rospy.is_shutdown():
    pub.publish(message)
    rate.sleep()
