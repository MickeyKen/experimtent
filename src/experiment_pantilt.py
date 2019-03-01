#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64,Int16
import time
from dynamixel_controllers.srv import SetSpeed

def callback(data):
    # print data.data
    pub_tilt = rospy.Publisher('tilt_controller/command', Float64, queue_size=10)
    exp_num = rospy.get_param("/exp_num")

    tilt_message = Float64()

    time.sleep(1)

    tilt_speed= 0.3


    if (exp_num == 1 or exp_num == 2 or exp_num == 3):
        tilt_speed= 0.5

    elif (exp_num == 4 or exp_num  == 5 or exp_num == 6):
        if exp_num == 4:
            tilt_speed= 0.5
        elif exp_num == 5:
            tilt_speed= 0.5
        else:
            tilt_speed= 0.5

    elif (exp_num == 7 or exp_num == 8 or exp_num == 9):
        if exp_num == 7:
            tilt_speed= 0.5
        elif exp_num == 8:
            tilt_speed= 0.5
        else:
            tilt_speed= 0.5
    else:
        pass

    set_tilt_speed = rospy.ServiceProxy('/tilt_controller/set_speed', SetSpeed)
    set_tilt_speed(tilt_speed)

    tilt_message = 0.5

    tilt_pub.publish(tilt_message)




def experiment_pantilt():

    rospy.init_node('experiment_for_pantilt', anonymous=True)

    rospy.Subscriber("finish_pantilt", Int16, callback)
    rospy.spin()

if __name__ == '__main__':
    experiment_pantilt()
