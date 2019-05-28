#!/usr/bin/env python
import cv2
import numpy as np
from std_msgs.msg import Int16MultiArray, Int16
from ubiquitous_display_pantilt.msg import Pantilt
import rospy
import time
import sys

def start_exp(exp_num):

    pantilt_pub = rospy.Publisher('pantilt_msg', Pantilt, queue_size=10)
    pantilt_radian_pub = rospy.Publisher('pantilt_radian_msg', Pantilt, queue_size=10)
    int_pub = rospy.Publisher('int', Int16, queue_size=10)
    rospy.init_node('start_exp', anonymous=True)
    pantilt_message = Pantilt()
    intten = Int16()
    intten = 9
    int_pub.publish(intten) 
    rospy.set_param("/exp_miki_img/switch", 0)

    pan = 0.5
    tilt = 0.5

    if exp_num == 0:
      rospy.set_param("/black_img/switch", 0)

    if exp_num == 1 or exp_num == 4 or exp_num == 7:
      pantilt_message.speed.x = pan
      pantilt_message.speed.y = tilt
      pantilt_message.speed.z = 0.0
      pantilt_message.position.x = 0.8
      pantilt_message.position.y = 0.72
      pantilt_message.position.z = 0.0
      pantilt_radian_pub.publish(pantilt_message)  
      rospy.loginfo(pantilt_message)
      #print "publish"
    if exp_num == 2 or exp_num == 5 or exp_num == 8:
      pantilt_message.speed.x = pan
      pantilt_message.speed.y = tilt
      pantilt_message.speed.z = 0.0
      pantilt_message.position.x = -1.21
      pantilt_message.position.y = 0.0
      pantilt_message.position.z = 0.0
      pantilt_pub.publish(pantilt_message)  
      rospy.loginfo(pantilt_message)
    if exp_num == 3  or exp_num == 6 or exp_num == 9:
      pantilt_message.speed.x = pan
      pantilt_message.speed.y = tilt
      pantilt_message.speed.z = 0.0
      pantilt_message.position.x = 1.21
      pantilt_message.position.y = 0.0
      pantilt_message.position.z = 0.0
      pantilt_pub.publish(pantilt_message)  
      rospy.loginfo(pantilt_message)
    if exp_num == 10:
      pantilt_message.speed.x = pan
      pantilt_message.speed.y = tilt
      pantilt_message.speed.z = 0.0
      pantilt_message.position.x = 0.0
      pantilt_message.position.y = 0.72
      pantilt_message.position.z = 0.0
      pantilt_radian_pub.publish(pantilt_message)  
      rospy.loginfo(pantilt_message)
      



if __name__ == '__main__':
    args = sys.argv
    #print len(args)

    if 2 == len(args):
      exp_num = int(args[1])
      #print exp_num
      rospy.set_param("/exp_num", exp_num)
      start_exp(exp_num)

    else:
      print('Arguments are not correct')

