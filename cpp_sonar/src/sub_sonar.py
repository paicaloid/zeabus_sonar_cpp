#! /usr/bin/python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Instantiate CvBridge
bridge = CvBridge()

def image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "mono8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        #cv2.imwrite('camera_image.jpeg', cv2_img)
        cv2.imshow('img',cv2_img)
        cv2.waitKey(30)
                

def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "imaging_sonar"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()