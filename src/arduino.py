#! /usr/bin/env python3

'''
This node communicates with the arduino.
It subcribes to the button states and publishes the displayed text and the LED states.
It also contains a state machine for the different motion settings
'''

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def com():

    rospy.init_node('arduino', anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        com()
    except rospy.ROSInterruptException:
        pass
