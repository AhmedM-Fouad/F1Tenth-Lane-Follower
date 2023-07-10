#!/usr/bin/env python

from __future__ import print_function
import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, LaserScan #import laserscan msg
import math
import numpy as np
import sys
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import Image as msg_Image
from cv_bridge import CvBridge, CvBridgeError
import os
import cv2


class detLane:
    def __init__(self):
        lidarscan_topic = '/scan'
        drive_topic = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'
        self.bridge = CvBridge()
	self.sub = rospy.Subscriber("/camera/color/image_raw", msg_Image, self.pid_control)
        #rospy.Subscriber('scan', LaserScan, self.lidar_callback)

        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
        self.sub = rospy.Subscriber('/camera/depth/image_rect_raw', msg_Image, self.imageDepthCallback)

    def imageDepthCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            pix = (data.width/2, data.height/2)
            sys.stdout.write('%s: Depth at center(%d, %d): %f(mm)\r' % ('/camera/depth/image_rect_raw', pix[0], pix[1], cv_image[pix[1], pix[0]]))
            sys.stdout.flush()
        except CvBridgeError as e:
            print(e)
            return


    def make_points(self, image, line):
        slope, intercept = line
        y1 = int(image.shape[0])
        y2 = int(y1*3/5)
        x1 = int((y1 - intercept)/slope)
        x2 = int((y2 - intercept)/slope)
        return [[x1, y1, x2, y2]]

    def average_slope_intercept(self, image, lines):
        left_fit    = []
        right_fit   = []
        if lines is None:
            return None
        for line in lines:
            for x1, y1, x2, y2 in line:
                fit = np.polyfit((x1,x2), (y1,y2), 1)
                slope = fit[0]
                intercept = fit[1]
                if slope < 0:
                    left_fit.append((slope, intercept))
                else:
                    right_fit.append((slope, intercept))

        if len(left_fit) and len(right_fit):

            left_fit_average  = np.average(left_fit, axis=0)
            right_fit_average = np.average(right_fit, axis=0)
            left_line  = self.make_points(image, left_fit_average)
            right_line = self.make_points(image, right_fit_average)
            averaged_lines = [left_line, right_line]
            return averaged_lines

    def canny(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        kernel = 5
        blur = cv2.GaussianBlur(gray,(kernel, kernel),0)
        canny = cv2.Canny(gray, 50, 150)
        return canny

    def display_lines(self, img,lines):
        line_image = np.zeros_like(img)
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image,(x1,y1),(x2,y2),(0,0,255),20)
                #print(x1)
        return line_image

    def region_of_interest(self, canny):
        height = canny.shape[0]
        width = canny.shape[1]
        mask = np.zeros_like(canny)

        triangle = np.array([[
        (45, height),
        (300, 200),
        (570, height),]], np.int32)

        cv2.fillPoly(mask, triangle, 255)
        masked_image = cv2.bitwise_and(canny, mask)
        return masked_image

    def pid_control(self, data):

	color_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
	canny_image = self.canny(color_image)
	cropped_canny = self.region_of_interest(canny_image)
	lines = cv2.HoughLinesP(cropped_canny, 2, np.pi/180, 100, np.array([]), minLineLength=40,maxLineGap=5)
	averaged_lines = self.average_slope_intercept(color_image, lines)
	line_image = self.display_lines(color_image, averaged_lines)
	ar = np.array(averaged_lines)
	steering_angle = 0
	#current_value = ar[0, 0]
	ar = ar.squeeze()
	ar = ar.reshape(1, -1)
	current_value = ar[0, 0]
	#print(current_value)

	if current_value < 80:
	    steering_angle = 0.1
	    #print(steering_angle)
	elif current_value > 140:
	    steering_angle = -0.1
	    #print(steering_angle)
	else:
	    steering_angle = 0
	#combo_image = cv2.addWeighted(color_image, 0.8, line_image, 1, 1)
	drive_msg = AckermannDriveStamped()
	drive_msg.header.stamp = rospy.Time.now()
	drive_msg.header.frame_id = "laser"
	drive_msg.drive.steering_angle = steering_angle
	drive_msg.drive.speed = 0.5
	self.drive_pub.publish(drive_msg)



def main(args):
    rospy.init_node("lane_follower_node", anonymous=True)
    dt = detLane()
    rospy.sleep(0.1)
    rospy.spin()


if __name__=='__main__':
    main(sys.argv)
