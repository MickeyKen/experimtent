#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64,Int16
import time

def callback(data):
    print data.data

    time.sleep(1)

    pub_tilt = rospy.Publisher('tilt_controller/command', Float64, queue_size=10)

    exp_num = rospy.get_param("/exp_num")






def experiment_pantilt():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('experiment_for_pantilt', anonymous=True)

    rospy.Subscriber("finish_pantilt", Int16, callback)
    rospy.spin()

if __name__ == '__main__':
    experiment_pantilt()
