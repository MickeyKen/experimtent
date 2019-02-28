#!/usr/bin/env python
import sys
import rospy
import tf
from std_msgs.msg import Float64
import time
from ubiquitous_display_pantilt.msg import Pantilt

def move(x, y):
    print x, y


def exp_control():
    rospy.init_node('exp_control', anonymous=True)

    ##### set initial tilt and pan radian #####
    position_1_tilt = 0.0
    position_1_pan = 0.0



    exp_num = 0
    exp_pos = 1
    current_pos = 1

    pantilt_radian_pub = rospy.Publisher('pantilt_radian_msg', Pantilt, queue_size=10)
    pub_pan = rospy.Publisher('pan_controller/command', Float64, queue_size=10)
    pub_tilt = rospy.Publisher('tilt_controller/command', Float64, queue_size=10)

    pantilt_message = Pantilt()
    float_pan = Float64()
    float_tilt = Float64()

    rospy.set_param("/exp_num",exp_num)
    rospy.set_param("/exp_pos",exp_pos)

    while True:
        a = input("Input exp_num: >>")
        # print a
        current_pos = rospy.get_param("exp_pos")

        pub_pan.publish(float_pan)
        pub_tilt.publish(float_tilt)
        time.sleep(10)

        if a == 1 or a == 4 or a == 7:
            rospy.set_param("/exp_num",a)

            if (current_pos == 1):
                pass
            elif (current_pos == 2):
                move(-2.5, -1.0)

            elif (current_pos == 3):
                move(-2.5, -3.7)

            else:
                pass
            rospy.set_param("/exp_pos",1)

            pantilt_message.speed.x = 0.5
            pantilt_message.speed.y = 0.5
            pantilt_message.speed.z = 0.0
            pantilt_message.position.x = 0.0
            pantilt_message.position.y = 0.72
            pantilt_message.position.z = 0.0
            pantilt_radian_pub.publish(pantilt_message)

        elif a == 2 or a == 5 or a == 8:
            rospy.set_param("/exp_num",a)

            if (current_pos == 1):
                move(2.5, 1.0)

            elif (current_pos == 2):
                pass

            elif (current_pos == 3):
                move(0, -2.7)

            else:
                pass
            rospy.set_param("/exp_pos",2)

            pantilt_message.speed.x = 0.5
            pantilt_message.speed.y = 0.5
            pantilt_message.speed.z = 0.0
            pantilt_message.position.x = 0.0
            pantilt_message.position.y = 0.72
            pantilt_message.position.z = 0.0
            pantilt_radian_pub.publish(pantilt_message)

        elif a == 3 or a == 6 or a == 9:
            rospy.set_param("/exp_num",a)

            if (current_pos == 1):
                move(2.5, 3.7)

            elif (current_pos == 2):
                move(0.0, 2.7)

            elif (current_pos == 3):
                pass

            else:
                pass
            rospy.set_param("/exp_pos",3)

            pantilt_message.speed.x = 0.5
            pantilt_message.speed.y = 0.5
            pantilt_message.speed.z = 0.0
            pantilt_message.position.x = 0.0
            pantilt_message.position.y = 0.72
            pantilt_message.position.z = 0.0
            pantilt_radian_pub.publish(pantilt_message)

        elif a == 0:
            sys.exit()

if __name__ == '__main__':
    exp_control()
