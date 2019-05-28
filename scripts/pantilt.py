#!/usr/bin/env python
import cv2
import numpy as np
from ubiquitous_display_pantilt.msg import Pantilt
from dynamixel_controllers.srv import SetSpeed
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64,Int16
from std_msgs.msg import Int16MultiArray
import rospy
import time
import math
import tf

class ListenerPantilt:
    def __init__(self):
        self.pantilt_sub = rospy.Subscriber('finish_pantilt', Int16, self.callback)
        self.pan_pub = rospy.Publisher('pan_controller/command', Float64, queue_size=10)
        self.tilt_pub = rospy.Publisher('tilt_controller/command', Float64, queue_size=10)
        self.listener = tf.TransformListener()
        self.step_length_90 = 0.4
        self.step_length_45 = 0.255
        self.ud_z_point = 1.21
        self.pan_message = Float64()
        self.tilt_message = Float64()
        self.sleep_time = 0.5

    def get_tilt_position():
        try:
          self.listener.waitForTransform('ud_base_footprint', '/ud_pt_projector_link', rospy.Time(), rospy.Duration(1.0))
          (trans,rot) = self.listener.lookupTransform('/ud_base_footprint', '/ud_pt_projector_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          rospy.loginfo("TF Exception")
          return
    
        e = tf.transformations.euler_from_quaternion((rot[0], rot[1], rot[2], rot[3]))
        #print "current sngle" ,math.degrees(e[0]),math.degrees(e[1]),math.degrees(e[2])
        #print trans[2]
        return e[0]

    def get_pan_position(self):
        try:
          self.listener.waitForTransform('ud_base_footprint', '/ud_pt_plate_link', rospy.Time(), rospy.Duration(1.0))
          (trans,rot) = self.listener.lookupTransform('/ud_base_footprint', '/ud_pt_plate_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          rospy.loginfo("TF Exception")
          return
    
        e = tf.transformations.euler_from_quaternion((rot[0], rot[1], rot[2], rot[3]))
        #print "current sngle" ,math.degrees(e[0]),math.degrees(e[1]),math.degrees(e[2])
        #print trans[2]
        return e[2]
        
    def callback(self, data):
        time.sleep(self.sleep_time)
        current_tilt = rospy.get_param("control_pantilt/current_tilt_degree")
        current_pan = rospy.get_param("control_pantilt/current_pan_degree")
        current_x = rospy.get_param("control_pantilt/current_x_position")
        current_y = rospy.get_param('control_pantilt/current_y_position')

        if data.data == 1:
          exp_num = rospy.get_param('/exp_num')
          print exp_num


          if exp_num == 1 or exp_num == 4 or exp_num == 7:
            print "start program of /exp_num: ", exp_num
            target_y = current_y
            set_pan_speed = rospy.ServiceProxy('/pan_controller/set_speed', SetSpeed)
            set_pan_speed(0.1)
            set_tilt_speed = rospy.ServiceProxy('/tilt_controller/set_speed', SetSpeed)
            set_tilt_speed(0.1)
              

            self.pan_message = 0.0
            self.tilt_message = 0.72
            self.pan_pub.publish(self.pan_message)
            self.tilt_pub.publish(self.tilt_message)
            
            while True:
              current_pan = self.get_pan_position() 
              if current_pan < 0.04 and current_pan > -0.04:
                break
              
            
          elif exp_num == 2 or exp_num == 5 or exp_num == 8:
            print "start program of /exp_num: ", exp_num 
            set_pan_speed = rospy.ServiceProxy('/pan_controller/set_speed', SetSpeed)
            set_pan_speed(0.2)
            set_tilt_speed = rospy.ServiceProxy('/tilt_controller/set_speed', SetSpeed)
            set_tilt_speed(0.03)
            
            for i in range(5):
              target_x = current_x + self.step_length_45
              target_y = current_y + self.step_length_45

              rad_pan = math.atan2(target_y, target_x)
              distance = math.sqrt(target_x*target_x + target_y*target_y)
              rad_tilt = math.atan2(self.ud_z_point, distance)
              self.pan_message = rad_pan - 1.5708
              self.tilt_message = rad_tilt
              self.pan_pub.publish(self.pan_message)
              self.tilt_pub.publish(self.tilt_message)

              while current_pan < rad_tilt + 0.02 and current_pan > rad_tilt - 0.02:
                current_pan = self.get_pan_position()

              current_x += self.step_length_45
              current_y += self.step_length_45

            print current_x, current_y


          elif exp_num == 3 or exp_num == 6 or exp_num == 9:
            print "start program of /exp_num: ", exp_num 
            set_pan_speed = rospy.ServiceProxy('/pan_controller/set_speed', SetSpeed)
            set_pan_speed(0.2)
            set_tilt_speed = rospy.ServiceProxy('/tilt_controller/set_speed', SetSpeed)
            set_tilt_speed(0.03)
            
            for i in range(5):
              target_x = current_x - self.step_length_45
              target_y = current_y + self.step_length_45

              rad_pan = math.atan2(target_y, target_x)
              distance = math.sqrt(target_x*target_x + target_y*target_y)
              rad_tilt = math.atan2(self.ud_z_point, distance)
              self.pan_message = rad_pan - 1.5708
              self.tilt_message = rad_tilt
              self.pan_pub.publish(self.pan_message)
              self.tilt_pub.publish(self.tilt_message)

              while current_pan < rad_tilt + 0.02 and current_pan > rad_tilt - 0.02:
                current_pan = self.get_pan_position()

              current_x -= self.step_length_45
              current_y += self.step_length_45
          else:
            print "error"
        print "finish pantlt.py from experiment_miki"  

def main():
    rospy.init_node('pantilt', anonymous=True)
    myPantilt = ListenerPantilt()
    rospy.spin()
if __name__ == '__main__':
    main()
