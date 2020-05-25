#!/usr/bin/env python

# We always need these lines in a ROS python node.  They import all of
# the basic ROS functions, and make sure that the Python path is set
# up properly.  If you cut and paste these lines, make sure you change
# the manifest name to point to the one in the package that you're
# writing.  ROS will use whatever manifest you specify, even if it's
# not in the current package.  This can be *really* hard to debug.
import roslib; roslib.load_manifest('color_detection')
import rospy
import numpy
import os
import Queue
# os.system(command)
import cv2, cv_bridge

# The image message
from sensor_msgs.msg import Image

# coor message with a bool for color detection and
# an array to store points msgs;
# points message that holds array of x,y coordinates
# for one segment (cone)
from color_detection.msg import coor, points

class colorDetector:
    def __init__(self):
        # create bridge
        self.bridge = cv_bridge.CvBridge()

        # Subscribe to image data
        self.image_sub = rospy.Subscriber('/camera/rgb/image_rect_color', Image, self.image_callback)

        # Publish coordinates to driver node
        self.coor_pub = rospy.Publisher('color_detected', coor, queue_size=100)

        # orange object coordinates stored in 2d array
        self.coor = coor()
        self.coor.colorDetected = False
        self.coor.segments = []

        self.points = points()
        self.points.points = []

        self.label = []



    def image_callback(self, msg):
        #image_raw to cv2
        image = self.bridge.imgmsg_to_cv2(msg)
        hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        blur = cv2.blur(hsv, (3, 3)) # blur the image

        #define color boundaries
        lower_yellow = numpy.array([99,139,199])
        upper_yellow = numpy.array([235,255,255])

        #find colors in boundaries and apply mask
        mask = cv2.inRange(blur, lower_yellow, upper_yellow)

        #if contour area of orange object exists
        M = cv2.moments(mask)
        if M['m00'] > 0:

            #print("orange object detected")
            self.coor.colorDetected = True

            height = len(mask)
            width = len(mask[0])

            segments = self.component_labeling(mask, height, width)

            self.fill_points(segments, height, width)

        else:

            self.coor.colorDetected = False
            self.coor.segments = []

        #cv2.imshow('hsv',image)
        #cv2.imshow('mask',mask)
        #cv2.waitKey()

        #publish coordinates
        self.coor_pub.publish(self.coor)

    def fill_points(self, segments, height, width):

        # add points messages to segments array
        for i in range(segments-1):
            tempPoints = points()
            tempPoints.points = []
            self.coor.segments.append(tempPoints)

        # fill points arrays with pixel coordinates
        for i in range(len(self.label)):
            for j in range(len(self.label[i])):
                #print self.label[i][j]
                if self.label[i][j] != 0:
                    contourNum = self.label[i][j] - 1

                    self.coor.segments[contourNum].points.append(i)
                    self.coor.segments[contourNum].points.append(j)


    # label different segments/objects
    def component_labeling(self, binary_image, height, width):
        q = Queue.Queue(height*width)
        self.label = [[0 for x in range(width)] for y in range(height)]

        current_label = 1
        for i in range(height):
            for j in range(width):

                # if pixel is white and unlabeled
                if binary_image[i][j] != 0 and self.label[i][j] == 0:

                    # label pixel
                    self.label[i][j] = current_label

                    # push onto queue
                    pixel = (i, j)
                    q.put(pixel)

                    # while q is not empty
                    while not q.empty():
                        pixel = q.get()
                        if pixel[0] != height-1 and pixel[1] != width-1:

                            # check neighboring pixels
                            # upper neighbor:
                            upper_neighbor = (pixel[0]-1, pixel[1])
                            if binary_image[upper_neighbor[0]][upper_neighbor[1]] != 0:
                                if self.label[upper_neighbor[0]][upper_neighbor[1]] == 0:
                                    self.label[upper_neighbor[0]][upper_neighbor[1]] = current_label
                                    q.put(upper_neighbor)

                            # lower neighbor
                            lower_neighbor = (pixel[0]+1, pixel[1])
                            #print 'lower neighbor', lower_neighbor
                            if binary_image[lower_neighbor[0]][lower_neighbor[1]] != 0:
                                if self.label[lower_neighbor[0]][lower_neighbor[1]] == 0:
                                    self.label[lower_neighbor[0]][lower_neighbor[1]] = current_label
                                    q.put(lower_neighbor)

                            # left neighbor
                            left_neighbor = (pixel[0], pixel[1]-1)
                            if binary_image[left_neighbor[0]][left_neighbor[1]] != 0:
                                if self.label[left_neighbor[0]][left_neighbor[1]] == 0:
                                    self.label[left_neighbor[0]][left_neighbor[1]] = current_label
                                    q.put(left_neighbor)

                            # right neighbor
                            right_neighbor = (pixel[0], pixel[1]+1)
                            if binary_image[right_neighbor[0]][right_neighbor[1]] != 0:
                                if self.label[right_neighbor[0]][right_neighbor[1]] == 0:
                                    self.label[right_neighbor[0]][right_neighbor[1]] = current_label
                                    q.put(right_neighbor)

                    current_label += 1

                else:
                    continue

        return current_label


if __name__ == "__main__":

    # Initialize the color node
    rospy.init_node('colorDetector', anonymous=True, log_level=rospy.DEBUG)

    colorDetector = colorDetector()

    rospy.spin()
