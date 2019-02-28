#!/usr/bin/env python
import sys
import rospy
import tf


def exp_control():
    rospy.init_node('exp_control', anonymous=True)
    #rospy.set_param("exp_miki_img/switch", 1)
    exp_num = 0
    rospy.set_param("/exp_num",exp_num)
    while True:
        a = input()
        # print a
        if a == 1:
            rospy.set_param("/exp_num",a)

        elif a == 2:
            rospy.set_param("/exp_num",a)

        elif a == 3:
            rospy.set_param("/exp_num",a)

        elif a == 4:
            rospy.set_param("/exp_num",a)

        elif a == 5:
            rospy.set_param("/exp_num",a)

        elif a == 6:
            rospy.set_param("/exp_num",a)

        elif a == 7:
            rospy.set_param("/exp_num",a)

        elif a == 8:
            rospy.set_param("/exp_num",a)

        elif a == 9:
            rospy.set_param("/exp_num",a)
            
        elif a == 0:
            break

if __name__ == '__main__':
    exp_control()
