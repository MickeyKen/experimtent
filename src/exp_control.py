#!/usr/bin/env python
import sys
import rospy
import tf
from std_msgs.msg import Float64
import time
from ubiquitous_display_pantilt.msg import Pantilt
from geometry_msgs.msg import Twist, Point, Quaternion
import PyKDL
from math import radians, copysign, sqrt, pow, pi



class Control():
    def __init__(self):

        pantilt_radian_pub = rospy.Publisher('pantilt_radian_msg', Pantilt, queue_size=10)
        pantilt_message = Pantilt()

        rospy.init_node('exp_control', anonymous=True)


        ##### set initial tilt and pan radian #####
        position_1_tilt = 0.0
        position_1_pan = 0.0

        exp_num = 0
        exp_pos = 1
        current_pos = 1

        pub_pan = rospy.Publisher('pan_controller/command', Float64, queue_size=10)
        pub_tilt = rospy.Publisher('tilt_controller/command', Float64, queue_size=10)

        float_pan = Float64()
        float_tilt = Float64()

        rospy.set_param("/exp_num",exp_num)
        rospy.set_param("/exp_pos",exp_pos)

        while True:
            a = input("Input exp_num: >>")
            # print a
            current_pos = rospy.get_param("exp_pos")
            rospy.set_param("/exp_miki_img/switch", 0)

            pub_pan.publish(float_pan)
            pub_tilt.publish(float_tilt)
            time.sleep(1)

            if a == 1 or a == 4 or a == 7 or a == 10:
                rospy.set_param("/exp_num",a)



                pantilt_message.speed.x = 0.5
                pantilt_message.speed.y = 0.5
                pantilt_message.speed.z = 0.0
                pantilt_message.position.x = 0.0
                pantilt_message.position.y = 1.1
                pantilt_message.position.z = 0.0
                pantilt_radian_pub.publish(pantilt_message)

            elif a == 2 or a == 5 or a == 8 or a == 11:
                rospy.set_param("/exp_num",a)



                pantilt_message.speed.x = 0.5
                pantilt_message.speed.y = 0.5
                pantilt_message.speed.z = 0.0
                pantilt_message.position.x = -0.78
                pantilt_message.position.y = 1.1
                pantilt_message.position.z = 0.0
                pantilt_radian_pub.publish(pantilt_message)


            elif a == 3 or a == 6 or a == 9 or a ==12:
                rospy.set_param("/exp_num",a)



                pantilt_message.speed.x = 0.5
                pantilt_message.speed.y = 0.5
                pantilt_message.speed.z = 0.0
                pantilt_message.position.x = -2.35
                pantilt_message.position.y = 1.1
                pantilt_message.position.z = 0.0
                pantilt_radian_pub.publish(pantilt_message)


            elif a == 0:
                sys.exit()
            else:
                pass



if __name__ == '__main__':
    Control()
