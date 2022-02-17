#!/usr/bin/env python3

#This script is responsible for image procesing visualisation

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rospy
from sensor_msgs.msg import Image

class Controller:
    def __init__(self):
        rospy.init_node('tomato_detector')

        #Subscriber
        self.image_sub = rospy.Subscriber("/camera/color/image_raw2", Image, self.image_callback)

        self.bridge = CvBridge()

    def image_callback(self, rgb_data):

        try:
            frame = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")

            #HSV conversion
            img_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            #upper and lower ranges for HSV masking
            low_HSV = np.array([0, 53, 137])
            upper_HSV = np.array([10, 255, 255])

            mask = cv2.inRange(img_HSV, low_HSV, upper_HSV)

            output = cv2.bitwise_and(frame, frame, mask = mask)
            output = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)

            ret, threshold = cv2.threshold(output, 25, 255, cv2.THRESH_BINARY)

            #contour formation
            contours, hierarchy = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            i=0

            for contour in contours:
                C = cv2.moments(contour)

                cX = int(C['m10']/(C['m00']+1e-4))
                cY = int(C['m01']/(C['m00']+1e-4))

                cv2.putText(frame, 'obj{}'.format(i),(cX, cY-50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.circle(frame, (cX, cY), 2, (255, 255, 255), -1)

                #creates the bounding boxes around the centres
                frame = cv2.rectangle(frame, (cX-30, cY-25), (cX+30, cY+25), (0, 255, 0), 2)
                i += 1

            cv2.imshow("camera_feed", frame)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    ct = Controller()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
