#!/usr/bin/env python
import cv2
import numpy as np
from std_msgs.msg import Int16MultiArray
import rospy
import time
import sys
from std_msgs.msg import Float64,Int16
import math
import tf

def get_pan_position():
    listener = tf.TransformListener()
    try:
      listener.waitForTransform('ud_base_footprint', '/ud_pt_plate_link', rospy.Time(), rospy.Duration(1.0))
      (trans,rot) = listener.lookupTransform('/ud_base_footprint', '/ud_pt_plate_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      rospy.loginfo("TF Exception")
      return
    e = tf.transformations.euler_from_quaternion((rot[0], rot[1], rot[2], rot[3]))
    return e[2]

def callback(data):

    img = cv2.imread('/home/ud/catkin_ws/src/experiment_miki/src/image/pop_90.png')
    img_blink = cv2.imread('/home/ud/catkin_ws/src/experiment_miki/src/image/pop_blink_90.png')
    h = 1024
    w = 768
    img = cv2.resize(img,(h,w))
    img_blink = cv2.resize(img_blink,(h,w))
    exp_num = rospy.get_param("/exp_num")
    rospy.set_param("exp_miki_img/switch", 1)
    print exp_num


    if exp_num == 8 or exp_num == 9:

      pts_src = np.array([[0,0], [1023,0], [1023,767], [0,767]],np.float32)
      pts_dst = np.array([[-530, 2300],[530, 2300],[355, 1160],[-355, 1160]],np.float32)

      cv2.namedWindow('window')
      cv2.namedWindow('screen', cv2.WINDOW_NORMAL)
      cv2.setWindowProperty('screen', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
 
      # Calculate Homography
      h, status = cv2.findHomography(pts_src, pts_dst)
      #print h#, status
      h = np.linalg.inv(h)
    

      while True:

        if rospy.get_param("exp_miki_img/switch") == 0:
          cv2.destroyAllWindows()
          break

        current_pan = get_pan_position()

        weight = (1.57 - current_pan) * 189

        new_pts_dst = np.array([[0, 0],[0, 0],[0, 0],[0, 0]],np.float32)

        

        f = np.float32([[-300.0, 1916.0, 1.0]])
        f = f.T
        f = np.dot(h, f)
        f = f / f[2][0]
        new_pts_dst[0][0] = f[0][0]
        new_pts_dst[0][1] = f[1][0]

        s = np.float32([[300.0, 1916.0, 1.0]])
        s = s.T
        s = np.dot(h, s)
        s = s / s[2][0]
        new_pts_dst[1][0] = s[0][0]
        new_pts_dst[1][1] = s[1][0]

        t = np.float32([[-300.0, 1316.0, 1.0]])
        t = t.T
        t = np.dot(h, t)
        t = t / t[2][0]
        new_pts_dst[3][0] = t[0][0]
        new_pts_dst[3][1] = t[1][0]

        fo = np.float32([[300.0, 1316.0, 1.0]])
        fo = fo.T
        fo = np.dot(h, fo)
        fo = fo / fo[2][0]
        new_pts_dst[2][0] = fo[0][0]
        new_pts_dst[2][1] = fo[1][0]

        M = cv2.getPerspectiveTransform(pts_src,new_pts_dst)
        if int(current_pan*10.0) % 2 ==  0:
          warp = cv2.warpPerspective(img, M, (1024,768))
        else:
          warp = cv2.warpPerspective(img_blink, M, (1024,768))

        cv2.imshow('screen', warp)
        cv2.waitKey(1)

      cv2.destroyAllWindows()


def exp_miki_img():
    rospy.init_node('exp_miki_img_89', anonymous=True)
    #rospy.set_param("exp_miki_img/switch", 1)
    rospy.Subscriber("finish_pantilt", Int16, callback) 
    rospy.spin()

if __name__ == '__main__':
    exp_miki_img()


