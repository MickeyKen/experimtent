#!/usr/bin/python

import rospy

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Int16

import message_filters

import cv2


bridge = CvBridge()


def callback(image, num):

    print("Received an image!")
    try:

        cv2_img = bridge.imgmsg_to_cv2(image, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:

        cv2.imwrite('camera_image ' + str(num) + '.jpeg', cv2_img)

def main():
    rospy.init_node('image_listener')

    image_sub = message_filters.Subscriber("/kinect2/hd/image_color_rect", Image)
    array_sub = message_filters.Subscriber("/shot", Int16)
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, array_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    main()
