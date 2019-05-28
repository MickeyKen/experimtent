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

    img = cv2.imread('/home/ud/catkin_ws/src/experiment_miki/src/image/pop.png')
    h = 1024
    w = 768
    img = cv2.resize(img,(h,w))
    exp_num = rospy.get_param("/exp_num")
    rospy.set_param("exp_miki_img/switch", 1)
    print exp_num

    if exp_num == 2 or exp_num == 3 or exp_num == 5 or exp_num == 6:
     
      if exp_num == 5 or exp_num == 6:
        black_image = np.zeros((678,1024,3),np.uint8)

      pts_src = np.array([[0,0], [1023,0], [1023,767], [0,767]])
      pts_dst = np.array([[-530, 2300],[530, 2300],[355, 1160],[-355, 1160]])
 
      # Calculate Homography
      h, status = cv2.findHomography(pts_src, pts_dst)
      #print h#, status
      h = np.linalg.inv(h)

      #get param
      x = 0.0
      y = 0.0

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

      pts_src = np.array([[0,0], [1023,0], [1023,767],[0,767]],np.float32)
      M = cv2.getPerspectiveTransform(pts_src,new_pts_dst)
      warp = cv2.warpPerspective(img, M, (1024,768))

      cv2.namedWindow('window')
      cv2.namedWindow('screen', cv2.WINDOW_NORMAL)
      cv2.setWindowProperty('screen', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
      cv2.imshow('screen', warp)
      cv2.waitKey(100)

      i = 0
 
      while True:
        i = i + 1
        if exp_num == 5 or exp_num == 6:
          if i % 2 == 0: 
            cv2.imshow('screen', black_image)
            cv2.waitKey(1)
            time.sleep(0.7)
          else:
            cv2.imshow('screen', warp)
            cv2.waitKey(1)
            time.sleep(0.7)
          
        if rospy.get_param("exp_miki_img/switch") == 0:
          break
          #sys.exit()
        else:
          pass

      cv2.destroyAllWindows()

    elif exp_num == 1:
      #"print pass"
      r = rospy.Rate(10) 
      pts_src = np.array([[0,0], [1023,0], [1023,767],[0,767]],np.float32)
      pts_org = np.array([[-620, 2660],[620, 2660],[390, 1320],[-390, 1320]])

      #print rospy.get_param("exp_miki_img/switch")

      cv2.namedWindow('window')
      cv2.namedWindow('screen', cv2.WINDOW_NORMAL)
      cv2.setWindowProperty('screen', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

      while True:
        #print rospy.get_param("exp_miki_img/switch")
        if rospy.get_param("exp_miki_img/switch") == 0:
          cv2.destroyAllWindows()
          break
        #print pts_org[0], pts_org[1]
        current_pan = get_pan_position()
        #deg = math.degrees(current_pan)
        #print "degree: ",deg
        #print "tan degree: " , math.tan(deg)
        centroid = -(1650 * math.tan(current_pan))
        #print "centroid: ", centroid
        rot = np.float32([[math.cos(current_pan), -(math.sin(current_pan))],
                          [math.sin(current_pan), math.cos(current_pan)]])

        new_pts_org = np.array([[0, 0],[0, 0],[0, 0],[0, 0]],np.float32)
        new_pts_org[0] = np.dot(rot, pts_org[0].T) 
        new_pts_org[1] = np.dot(rot, pts_org[1].T)
        new_pts_org[2] = np.dot(rot, pts_org[2].T)
        new_pts_org[3] = np.dot(rot, pts_org[3].T)

        # Calculate Homography
        h, status = cv2.findHomography(pts_src, new_pts_org)
        #print h#, status
        h = np.linalg.inv(h)

        f = np.float32([[centroid - 300.0, 1950.0, 1.0]])
        f = f.T
        f = np.dot(h, f)
        f = f / f[2][0]
        #print f
        new_pts_org[0][0] = f[0][0]
        new_pts_org[0][1] = f[1][0]

        s = np.float32([[centroid + 300.0, 1950.0, 1.0]])
        s = s.T
        s = np.dot(h, s)
        s = s / s[2][0]
        #print s
        new_pts_org[1][0] = s[0][0]
        new_pts_org[1][1] = s[1][0]
        
        t = np.float32([[centroid - 300.0, 1350.0, 1.0]])
        t = t.T
        t = np.dot(h, t)
        t = t / t[2][0]
        #print t
        new_pts_org[3][0] = t[0][0]
        new_pts_org[3][1] = t[1][0]

        fo = np.float32([[centroid + 300.0, 1350.0, 1.0]])
        fo = fo.T
        fo = np.dot(h, fo)
        fo = fo / fo[2][0]
        #print fo
        new_pts_org[2][0] = fo[0][0]
        new_pts_org[2][1] = fo[1][0]

        M = cv2.getPerspectiveTransform(pts_src,new_pts_org)
        warp = cv2.warpPerspective(img, M, (1024,768))

        cv2.imshow('screen', warp)
        #print warp
        #print "pass"
        cv2.waitKey(1)
        #time.sleep(1)
      cv2.destroyAllWindows()


def exp_miki_img():
    rospy.init_node('exp_miki_img', anonymous=True)
    #rospy.set_param("exp_miki_img/switch", 1)
    rospy.Subscriber("finish_pantilt", Int16, callback) 
    rospy.spin()

if __name__ == '__main__':
    exp_miki_img()


