#!/usr/bin/env python
import sys
import rospy
import tf
import time
from ubiquitous_display_pantilt.msg import Pantilt
from math import radians, copysign, sqrt, pow, pi
from std_msgs.msg import Float64

def change_controller():

    rospy.init_node('change_controller', anonymous=True)

    pantilt_radian_pub = rospy.Publisher('pantilt_radian_msg', Pantilt, queue_size=10)
    pantilt_message = Pantilt()
    pub_pan = rospy.Publisher('pan_controller/command', Float64, queue_size=10)
    pub_tilt = rospy.Publisher('tilt_controller/command', Float64, queue_size=10)

    float_pan = Float64()
    float_tilt = Float64()

    while True:
        a = input("Input exp_num: >>")

        rospy.set_param("/exp_num",a)

        float_tilt = 1.1
        pub_tilt.publish(float_tilt)

        rospy.set_param("/exp_miki_img/switch",0)

        time.sleep(2)
        if a == 1 or a == 4 or a == 7 or a == 10:

            pantilt_message.speed.x = 0.5
            pantilt_message.speed.y = 0.5
            pantilt_message.speed.z = 0.0
            pantilt_message.position.x = 0.0
            pantilt_message.position.y = 1.1
            pantilt_message.position.z = 0.0
            pantilt_radian_pub.publish(pantilt_message)

        elif a == 2 or a == 5 or a == 8 or a == 11:

            pantilt_message.speed.x = 0.5
            pantilt_message.speed.y = 0.5
            pantilt_message.speed.z = 0.0
            pantilt_message.position.x = -0.78
            pantilt_message.position.y = 1.1
            pantilt_message.position.z = 0.0
            pantilt_radian_pub.publish(pantilt_message)

        elif a == 3 or a == 6 or a == 9 or a == 12:
            pantilt_message.speed.x = 0.5
            pantilt_message.speed.y = 0.5
            pantilt_message.speed.z = 0.0
            pantilt_message.position.x = -2.38
            pantilt_message.position.y = 1.1
            pantilt_message.position.z = 0.0
            pantilt_radian_pub.publish(pantilt_message)
        elif a == 0:
            float_tilt = 0.0
            pub_tilt.publish(float_tilt)
            sys.exit()
        else:
            pass
if __name__ == '__main__':
    change_controller()
